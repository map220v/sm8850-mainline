// SPDX-License-Identifier: GPL-2.0-only
/*
 * oplus-camres — camera shared I2C bus (QUP wrap2 SE1) between AP and SEE
 *
 * gpio4/5 are physically shared between the AP and the SEE (sensors
 * execution environment on the ADSP), which uses the bus for AON and
 * virtual camera sensors.  This driver exposes the bus as its own I2C
 * adapter ("oplus,camres"): camera sensor/VCM/EEPROM children hang off
 * it in DT.  Transfers are forwarded to a geni_i2c platform device that
 * is instantiated from the (otherwise disabled) i2c9 DT node only while
 * the AP holds the bus — geni_i2c itself stays unmodified.
 *
 * Ownership:
 *
 *   - SEE (default): nothing in DT occupies gpio4/5, i2c9 stays
 *     disabled, no geni device exists.  Transfers return -EBUSY.
 *
 *   - AP: a resident SNS client negotiates with SEE, then the pins are
 *     muxed to qup2_se1 and the geni device is instantiated:
 *
 *       registration (once, at service discovery):
 *           SUID lookup (msg 512, magic uid) -> 768
 *       session bring-up (retried until SEE answers; the AON subscribe
 *           powers the camera module — once, and kept standing):
 *           INIT 0x240 {name, 0, slot}   -> ACK 0x340 (error==0)
 *           RELEASE 0x241 {0}            -> ACK 0x341
 *           AON subscribe 516 + 513{cfg} -> camera_oplus_aon, ACK 0x20D
 *       takeover:  REVOKE 0x243 {}        -> ACK 0x343
 *                  [only while the AON subscribe stands: SEE does not
 *                   answer REVOKE, and AON commands die after it]
 *       release:   RELEASE 0x241 {0}      -> ACK 0x341
 *
 *     When INIT is ACKed with an error instead (module .pb missing)
 *     there is no session to arbitrate: the AON subscribe still runs
 *     (it powers the module), but the bus is taken once and held to
 *     the AP for good — acquire/release become no-ops until SEE
 *     restarts and the classification is re-made.
 *
 *     The sequence is ACK-driven: each request blocks until its ACK
 *     indication arrives (or times out); there are no sleeps.
 *
 * All SNS/QRTR/QMI wire layouts are fixed on this firmware and are
 * described by packed structs that are validated strictly — a datagram
 * that does not match is dropped with a reason and hex logged.
 *
 * sysfs:
 *   force_start_i2c        — kernel negotiates with SEE, then takes
 *                            the bus.  Write 0 to drop the hold.
 *   ap_holds               — read current ownership (1 = AP)
 */

#include <linux/delay.h>
#include <linux/capability.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/net.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/qrtr.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <net/sock.h>
#include <linux/uio.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <linux/oplus-camres.h>

/* ---- protocol constants (validated against libqmi/libqrtr + SEE) ---- */

#define SNS_SERVICE_ID		400
#define SNS_QMI_CONTROL		0x0020	/* QMI SSC "Control" request */
#define SNS_QMI_REPORT_SMALL	0x0021
#define SNS_QMI_REPORT_LARGE	0x0022

#define SNS_MSG_SUID_LOOKUP	512
#define SNS_MSG_SUID_RESULT	768
#define SNS_SUID_MAGIC		0xababababababababULL

/* camera_oplus_aon subscribe (captured vendor sequence; on the wire
 * these are the qsh_camera_oplus 0x204 algo query and 0x201 sensor
 * subscribe): attributes request (516, empty), then config (513) with
 * {320x240 @ 5 fps} — this is what powers the camera module, and its
 * ACK 0x20D (error==0) is the precondition for REVOKE. */
#define SNS_MSG_SENSOR_INFO	516
#define SNS_MSG_SENSOR_CONFIG	513
#define SNS_MSG_SENSOR_CONFIG_ACK 0x20D
#define SNS_AON_UID_LO		0xf8447cc60209c100ULL
#define SNS_AON_UID_HI		0x3e9145ace8717480ULL

#define QSH_INIT		0x240	/* attach {name, retry, slot, otp} */
#define QSH_INIT_ACK		0x340
#define QSH_RELEASE		0x241	/* hand back to SEE {power} */
#define QSH_RELEASE_ACK		0x341
#define QSH_REVOKE		0x243	/* take from SEE {} */
#define QSH_REVOKE_ACK		0x343

#define ACK_TIMEOUT_MS		2000
#define GENI_TIMEOUT_MS		2000
#define SUSPEND_TIMEOUT_MS	1000
#define LOOKUP_RETRY_MS		3000
#define LOOKUP_REPLAY_MS	300
#define SESSION_RETRY_MS	1000

#define CAMRES_ATTACH_NAME	"icelandmain"
#define CAMRES_ATTACH_SLOT	0
#define CAMRES_SSC_MAX		1024

/* protobuf tags: (field << 3) | wire type */
#define PB_BYTES(f)	(((f) << 3) | 2)
#define PB_F64(f)	(((f) << 3) | 1)
#define PB_F32(f)	(((f) << 3) | 5)
#define PB_VARINT(f)	(((f) << 3) | 0)

/* ---- SNS wire structs (fixed layouts, little-endian) ---- */

/* QMI SSC Control request SDU */
struct camres_qmi_req {
	u8 flags;			/* 0x00 = request */
	__le16 txn;
	__le16 msg_id;			/* SNS_QMI_CONTROL */
	__le16 msg_len;
	u8 report_type;			/* TLV 0x10 */
	__le16 report_len;		/* 1 */
	u8 report;			/* 1 = large reports */
	u8 data_type;			/* TLV 0x01 */
	__le16 data_len;		/* 2 + pb_len */
	__le16 pb_len;
	u8 pb[];			/* SscClientRequest protobuf */
} __packed;

/* QMI SSC Report indication SDU */
struct camres_qmi_ind {
	u8 flags;			/* 0x04 = indication */
	__le16 txn;
	__le16 msg_id;			/* SNS_QMI_REPORT_LARGE */
	__le16 msg_len;
	u8 client_type;			/* TLV 0x01 */
	__le16 client_len;		/* 8 */
	u8 client_id[8];
	u8 data_type;			/* TLV 0x02 */
	__le16 data_len;		/* 2 + pb_len */
	__le16 pb_len;
	u8 pb[];			/* SscClientResponse protobuf */
} __packed;

/* SscClientRequest{1: uid, 2: msg_id, 3: config, 4: body{2: msg}} */
struct camres_sns_req {
	u8 uid_tag;			/* PB_BYTES(1) */
	u8 uid_len;			/* 18 */
	u8 lo_tag;			/* PB_F64(1) */
	__le64 uid_lo;
	u8 hi_tag;			/* PB_F64(2) */
	__le64 uid_hi;
	u8 msg_id_tag;			/* PB_F32(2) */
	__le32 msg_id;
	u8 cfg_tag;			/* PB_BYTES(3) */
	u8 cfg_len;			/* 4 */
	u8 cfg[4];			/* {1:1, 2:0} */
	u8 body_tag;			/* PB_BYTES(4) */
	u8 body_len;			/* 2 + msg_len */
	u8 msg_tag;			/* PB_BYTES(2) */
	u8 msg_len;
	u8 msg[];
} __packed;

/*
 * SscClientResponse{1: uid, 2: body{1: msg_id, 2: ts, 3: {1: error,
 * 2: {1: state}}}} — handshake ACK shape.  The SUID registry reply
 * shares the envelope and body.1 but has a different tail, so the tail
 * is validated only when body_len says it is a handshake ACK.
 */
struct camres_sns_resp {
	u8 uid_tag;			/* PB_BYTES(1) */
	u8 uid_len;			/* 18 */
	u8 lo_tag;			/* PB_F64(1) */
	u8 uid_lo[8];
	u8 hi_tag;			/* PB_F64(2) */
	u8 uid_hi[8];
	u8 body_tag;			/* PB_BYTES(2) */
	u8 body_len;
	u8 msg_id_tag;			/* PB_F32(1) */
	__le32 msg_id;
	/* handshake tail follows (see camres_sns_ack_tail) */
} __packed;

struct camres_sns_ack_tail {
	u8 ts_tag;			/* PB_F64(2) */
	u8 ts[8];
	u8 err_tag;			/* PB_BYTES(3) */
	u8 err_len;			/* 6 */
	u8 error_tag;			/* PB_VARINT(1) */
	u8 error;
	u8 state_tag;			/* PB_BYTES(2) */
	u8 state_len;			/* 2 */
	u8 state_val_tag;		/* PB_VARINT(1) */
	u8 state;
} __packed;

#define ACK_TAIL_LEN		(5 + sizeof(struct camres_sns_ack_tail))

/*
 * REVOKE ack tail (0x343): same {2: ts, 3: {...}} envelope as the
 * other handshake acks, but field 3 carries {1: error, 5..8:
 * optional AON stats blobs} instead of a nested power/state
 * submessage (captured: body 22, err_len 6 = "08 00" + "3a 02 08 00").
 * Only the fixed prefix is validated; the stats blobs are ignored.
 */
struct camres_sns_revoke_tail {
	u8 ts_tag;			/* PB_F64(2) */
	u8 ts[8];
	u8 err_tag;			/* PB_BYTES(3) */
	u8 err_len;			/* >= 2, grows with stats blobs */
	u8 error_tag;			/* PB_VARINT(1) */
	u8 error;
} __packed;

/* ---- driver state ---- */

struct oplus_camres {
	struct device *dev;

	struct mutex lock;
	atomic_t consumers;
	bool force;		/* sysfs hold active */
	bool ap_active;		/* pins muxed + geni instantiated */

	struct pinctrl *pctl;
	struct pinctrl_state *p_active;
	struct pinctrl_state *p_released;

	struct device_node *geni_np;	/* the disabled i2c9 DT node */
	struct platform_device *geni_pdev;
	struct i2c_adapter *geni_adap;
	struct device *geni_dev;

	struct i2c_adapter adapter;	/* the oplus,camres bus itself */
	struct delayed_work release_work;

	/* resident SNS client */
	struct socket *sock;
	void (*saved_data_ready)(struct sock *sk);
	struct work_struct rx_work;	/* drains the socket on data-ready */
	wait_queue_head_t rx_wq;	/* discovery: sns_found changes */
	struct mutex sns_lock;		/* one request/ack transaction at a time */
	spinlock_t ack_lock;		/* the armed-ack state below */
	struct completion ack_done;
	u32 ack_expect;			/* msg id awaited, 0 = none */
	u64 ack_uid_lo;
	u64 ack_uid_hi;
	struct sockaddr_qrtr sns_sq;
	bool sns_found;
	bool registered;	/* SNS client registered (ADSP up) */
	bool hs_ready;		/* INIT answered once (bring-up step 1) */
	bool hs_ok;		/* handshake session valid (INIT error==0) */
	bool aon_armed;		/* AON subscribe acked — REVOKE precondition */
	bool session_ready;	/* SEE camera session up (module powered) */
	u16 txn;
	u64 suid_hi;
	u64 suid_lo;
	char attach_name[24];
	u32 attach_slot;
	u8 ack_error;		/* from the last ACK */
	u8 ack_state;
	struct delayed_work lookup_work;
	struct delayed_work session_work;
};

static struct oplus_camres *g_camres;

/* experiment knobs (defaults come from DT) */
static bool revoke = true;
module_param(revoke, bool, 0644);
static u64 suid_hi, suid_lo;
module_param(suid_hi, ullong, 0444);
module_param(suid_lo, ullong, 0444);
static char *attach_name;
module_param(attach_name, charp, 0444);
static int attach_slot = -1;
module_param(attach_slot, int, 0444);

/* ---- protobuf emitters (only for the variable-length name field) ---- */

static size_t pb_put_bytes(u8 *b, size_t pos, unsigned int field,
			   const void *data, size_t len)
{
	b[pos++] = PB_BYTES(field);
	b[pos++] = len;			/* names are short (< 128) */
	if (len)
		memcpy(b + pos, data, len);
	return pos + len;
}

static size_t pb_put_varint(u8 *b, size_t pos, unsigned int field, u64 val)
{
	b[pos++] = PB_VARINT(field);
	b[pos++] = val;			/* values used here are < 128 */
	return pos;
}

/*
 * Message bodies:
 *   SUID lookup: {1: "camera_handshake", 2: 0, 3: 1}
 *   INIT:        {1: name, 2: 0, 3: slot, 4: ""}
 *   RELEASE:     {1: 0}
 *   REVOKE:      {} (empty)
 */
static size_t sns_body_lookup(u8 *b)
{
	size_t pos = 0;

	pos = pb_put_bytes(b, pos, 1, "camera_handshake", 16);
	pos = pb_put_varint(b, pos, 2, 0);
	pos = pb_put_varint(b, pos, 3, 1);
	return pos;
}

static size_t sns_body_init(u8 *b, const char *name, u32 slot)
{
	size_t pos = 0;

	pos = pb_put_bytes(b, pos, 1, name, strlen(name));
	pos = pb_put_varint(b, pos, 2, 0);
	pos = pb_put_varint(b, pos, 3, slot);
	pos = pb_put_bytes(b, pos, 4, NULL, 0);
	return pos;
}

/* RELEASE_CONTROL body {1: 0} */
struct camres_sns_body_release {
	u8 power_tag;		/* PB_VARINT(1) */
	u8 power;		/* 0 = hand back to SEE */
} __packed;

/* ---- SNS transport ---- */

static void camres_data_ready(struct sock *sk);

static int sns_socket_open(struct oplus_camres *cr)
{
	struct sock *sk;
	int ret;

	if (cr->sock)
		return 0;

	ret = sock_create_kern(&init_net, AF_QIPCRTR, SOCK_DGRAM, 0,
			       &cr->sock);
	if (ret) {
		dev_err(cr->dev, "qrtr socket: %d\n", ret);
		return ret;
	}

	/* event-driven receive, as net/qrtr/ns.c does for its socket */
	sk = cr->sock->sk;
	write_lock_bh(&sk->sk_callback_lock);
	sk->sk_user_data = cr;
	cr->saved_data_ready = sk->sk_data_ready;
	sk->sk_data_ready = camres_data_ready;
	write_unlock_bh(&sk->sk_callback_lock);
	return 0;
}

static void sns_socket_close(struct oplus_camres *cr)
{
	struct sock *sk;

	if (!cr->sock)
		return;

	sk = cr->sock->sk;
	write_lock_bh(&sk->sk_callback_lock);
	sk->sk_data_ready = cr->saved_data_ready;
	sk->sk_user_data = NULL;
	write_unlock_bh(&sk->sk_callback_lock);

	cancel_work_sync(&cr->rx_work);
	sock_release(cr->sock);
	cr->sock = NULL;
}

static int sns_send(struct oplus_camres *cr, u64 uid_lo, u64 uid_hi,
		    u32 msg_id, const u8 *msg, size_t msg_len)
{
	u8 pb_buf[sizeof(struct camres_sns_req) + 32];
	u8 pkt_buf[sizeof(pb_buf) + sizeof(struct camres_qmi_req)];
	struct camres_sns_req *pb = (void *)pb_buf;
	struct camres_qmi_req *sdu = (void *)pkt_buf;
	struct sockaddr_qrtr to = cr->sns_sq;
	struct msghdr m = { .msg_name = &to, .msg_namelen = sizeof(to) };
	struct kvec iov;
	size_t pkt_len;
	int ret;

	if (!cr->sns_found)
		return -ENOTCONN;
	if (msg_len > sizeof(pb_buf) - sizeof(*pb))
		return -EMSGSIZE;

	ret = sns_socket_open(cr);
	if (ret)
		return ret;

	*pb = (struct camres_sns_req){
		.uid_tag = PB_BYTES(1),	.uid_len = 18,
		.lo_tag = PB_F64(1),	.uid_lo = cpu_to_le64(uid_lo),
		.hi_tag = PB_F64(2),	.uid_hi = cpu_to_le64(uid_hi),
		.msg_id_tag = PB_F32(2), .msg_id = cpu_to_le32(msg_id),
		.cfg_tag = PB_BYTES(3), .cfg_len = 4,
		.cfg = { PB_VARINT(1), 1, PB_VARINT(2), 0 },
		.body_tag = PB_BYTES(4), .body_len = 2 + msg_len,
		.msg_tag = PB_BYTES(2), .msg_len = msg_len,
	};
	if (msg_len)
		memcpy(pb->msg, msg, msg_len);

	sdu->flags = 0x00;
	sdu->txn = cpu_to_le16(cr->txn++);
	sdu->msg_id = cpu_to_le16(SNS_QMI_CONTROL);
	sdu->report_type = 0x10;
	sdu->report_len = cpu_to_le16(1);
	sdu->report = 1;				/* large */
	sdu->data_type = 0x01;
	sdu->pb_len = cpu_to_le16(sizeof(*pb) + msg_len);
	sdu->data_len = cpu_to_le16(sizeof(*pb) + msg_len + 2);
	sdu->msg_len = cpu_to_le16(4 + 3 + 2 + sizeof(*pb) + msg_len);
	pkt_len = sizeof(*sdu) + sizeof(*pb) + msg_len;
	memcpy(sdu->pb, pb_buf, sizeof(*pb) + msg_len);

	iov.iov_base = pkt_buf;
	iov.iov_len = pkt_len;

	ret = kernel_sendmsg(cr->sock, &m, &iov, 1, pkt_len);
	if (ret < 0)
		dev_err(cr->dev, "sns send 0x%x: %d\n", msg_id, ret);
	return ret < 0 ? ret : 0;
}

/* control-plane packets are handled by the discovery parser instead */
static bool camres_parse_ctrl(struct oplus_camres *cr, const u8 *buf,
			      size_t len);

/*
 * Validate one report indication as a response for @uid carrying
 * @expect.  Handshake ACKs also yield error/power-state; registry
 * replies only match on msg_id.  Returns a negative reason code when
 * the datagram does not match (for logging), 0 on match.
 */
static int sns_parse_ack(const u8 *buf, size_t len, u64 uid_lo, u64 uid_hi,
			 u32 expect, struct camres_sns_ack_tail *tail)
{
	const struct camres_qmi_ind *ind = (const void *)buf;
	const struct camres_sns_resp *resp;
	size_t pb_len;

	if (len < sizeof(*ind) + sizeof(*resp))
		return -1;
	if (ind->flags != 0x04)
		return -2;
	if (ind->msg_id != cpu_to_le16(SNS_QMI_REPORT_LARGE) &&
	    ind->msg_id != cpu_to_le16(SNS_QMI_REPORT_SMALL))
		return -3;
	if (ind->client_type != 0x01 || ind->client_len != cpu_to_le16(8))
		return -4;
	if (ind->data_type != 0x02)
		return -5;
	pb_len = le16_to_cpu(ind->pb_len);
	if (ind->data_len != cpu_to_le16(pb_len + 2))
		return -6;
	if (len != sizeof(*ind) + pb_len)
		return -7;
	if (pb_len < sizeof(*resp))
		return -8;

	resp = (const void *)ind->pb;
	if (resp->uid_tag != PB_BYTES(1) || resp->uid_len != 18 ||
	    resp->lo_tag != PB_F64(1) || resp->hi_tag != PB_F64(2))
		return -9;
	if (get_unaligned_le64(resp->uid_lo) != uid_lo ||
	    get_unaligned_le64(resp->uid_hi) != uid_hi)
		return -10;
	if (resp->body_tag != PB_BYTES(2) || resp->msg_id_tag != PB_F32(1))
		return -11;
	if (le32_to_cpu(resp->msg_id) != expect)
		return -12;

	if (resp->body_len == ACK_TAIL_LEN) {
		const struct camres_sns_ack_tail *t;

		if (pb_len != sizeof(*resp) + sizeof(*t))
			return -13;
		t = (const void *)(resp + 1);
		if (t->ts_tag == PB_F64(2) && t->err_tag == PB_BYTES(3) &&
		    t->err_len == 6 && t->error_tag == PB_VARINT(1) &&
		    t->state_tag == PB_BYTES(2) && t->state_len == 2 &&
		    t->state_val_tag == PB_VARINT(1)) {
			*tail = *t;
			return 0;
		}
		if (expect != QSH_REVOKE_ACK)
			return -14;
	}

	if (expect == QSH_REVOKE_ACK) {
		const struct camres_sns_revoke_tail *t;

		/* {2: ts, 3: {1: error, 5..8: optional stats}} */
		if (resp->body_len < 5 + sizeof(*t) ||
		    pb_len < sizeof(*resp) + sizeof(*t))
			return -15;
		t = (const void *)(resp + 1);
		if (t->ts_tag != PB_F64(2) || t->err_tag != PB_BYTES(3) ||
		    t->err_len < 2 || t->error_tag != PB_VARINT(1))
			return -16;
		memset(tail, 0, sizeof(*tail));
		tail->error = t->error;
		return 0;
	}

	/* registry reply shape: no error/state tail */
	memset(tail, 0, sizeof(*tail));
	return 0;
}

/* softirq context: just kick the drain work */
static void camres_data_ready(struct sock *sk)
{
	struct oplus_camres *cr;

	read_lock_bh(&sk->sk_callback_lock);
	cr = sk->sk_user_data;
	if (cr)
		schedule_work(&cr->rx_work);
	read_unlock_bh(&sk->sk_callback_lock);
}

/* a data-plane datagram: complete the armed transaction if it matches */
static void camres_rx_ack(struct oplus_camres *cr, const u8 *buf, size_t len)
{
	struct camres_sns_ack_tail tail;
	int ret;

	spin_lock(&cr->ack_lock);
	if (!cr->ack_expect) {
		ret = -ENOENT;
		goto out;
	}
	ret = sns_parse_ack(buf, len, cr->ack_uid_lo, cr->ack_uid_hi,
			    cr->ack_expect, &tail);
	if (ret)
		goto out;

	cr->ack_error = tail.error;
	cr->ack_state = tail.state;
	dev_info(cr->dev, "sns ack 0x%x: error=%u state=%u\n",
		 cr->ack_expect, tail.error, tail.state);
	cr->ack_expect = 0;
	complete(&cr->ack_done);
out:
	spin_unlock(&cr->ack_lock);
	if (ret) {
		dev_dbg(cr->dev, "sns datagram dropped (reason %d)\n", ret);
		print_hex_dump_debug("sns drop: ", DUMP_PREFIX_OFFSET, 16, 1,
				     buf, min_t(size_t, len, 128), true);
	}
}

/*
 * Drain everything queued on the socket.  Control-plane datagrams go
 * to the discovery parser, the rest to the ack matcher.  Runs whenever
 * the socket signals data-ready, so nobody has to poll.
 */
static void rx_work_fn(struct work_struct *work)
{
	struct oplus_camres *cr = container_of(work, struct oplus_camres,
					       rx_work);
	u8 *buf __free(kfree) = kmalloc(CAMRES_SSC_MAX + 32, GFP_KERNEL);
	int ret;

	if (!buf)
		return;

	while (1) {
		struct sockaddr_qrtr from = { };
		struct msghdr m = { .msg_name = &from,
				    .msg_namelen = sizeof(from) };
		struct kvec iov = { buf, CAMRES_SSC_MAX + 31 };

		ret = kernel_recvmsg(cr->sock, &m, &iov, 1, iov.iov_len,
				     MSG_DONTWAIT);
		if (ret <= 0)
			break;

		if (from.sq_port == QRTR_PORT_CTRL) {
			camres_parse_ctrl(cr, buf, ret);
			wake_up(&cr->rx_wq);
		} else {
			camres_rx_ack(cr, buf, ret);
		}
	}
}

/*
 * One request/ack transaction: arm the matcher, send, wait.  Arming
 * before the send closes the window where a fast ack could arrive
 * before anyone expects it.
 */
static int sns_xfer(struct oplus_camres *cr, u64 uid_lo, u64 uid_hi,
		    u32 msg_id, const u8 *msg, size_t msg_len, u32 expect)
{
	int ret;

	mutex_lock(&cr->sns_lock);

	spin_lock(&cr->ack_lock);
	reinit_completion(&cr->ack_done);
	cr->ack_expect = expect;
	cr->ack_uid_lo = uid_lo;
	cr->ack_uid_hi = uid_hi;
	spin_unlock(&cr->ack_lock);

	ret = sns_send(cr, uid_lo, uid_hi, msg_id, msg, msg_len);
	if (!ret && !wait_for_completion_timeout(&cr->ack_done,
					msecs_to_jiffies(ACK_TIMEOUT_MS)))
		ret = -ETIMEDOUT;

	spin_lock(&cr->ack_lock);
	cr->ack_expect = 0;
	spin_unlock(&cr->ack_lock);

	mutex_unlock(&cr->sns_lock);
	return ret;
}

/* ---- resident client: discovery + registration ---- */

/*
 * QRTR service discovery, libqrtr semantics: a QRTR_TYPE_NEW_LOOKUP
 * with an all-zero filter on our own socket; the in-kernel name service
 * replays every registered server and announces later ones.  We select
 * the SNS service ourselves.
 */
static void lookup_work_fn(struct work_struct *work)
{
	struct oplus_camres *cr = container_of(to_delayed_work(work),
					       struct oplus_camres,
					       lookup_work);
	struct qrtr_ctrl_pkt pkt = { };
	struct sockaddr_qrtr to = { };
	struct msghdr m = { .msg_name = &to, .msg_namelen = sizeof(to) };
	struct kvec iov = { &pkt, sizeof(pkt) };
	int ret;

	if (cr->sns_found)
		return;

	ret = sns_socket_open(cr);
	if (ret)
		goto retry;

	ret = kernel_getsockname(cr->sock, (struct sockaddr *)&to);
	if (ret < 0)
		goto retry;
	to.sq_family = AF_QIPCRTR;
	to.sq_port = QRTR_PORT_CTRL;
	pkt.cmd = cpu_to_le32(QRTR_TYPE_NEW_LOOKUP);
	ret = kernel_sendmsg(cr->sock, &m, &iov, 1, sizeof(pkt));
	if (ret < 0)
		goto retry;

	/* rx_work parses the replay; wait for it to spot the SNS service */
	if (!wait_event_timeout(cr->rx_wq, cr->sns_found,
				msecs_to_jiffies(LOOKUP_REPLAY_MS)))
		goto retry;

	/* register the resident client with the SUID registry */
	{
		u8 body[32];
		size_t len = sns_body_lookup(body);

		ret = sns_xfer(cr, SNS_SUID_MAGIC, SNS_SUID_MAGIC,
			       SNS_MSG_SUID_LOOKUP, body, len,
			       SNS_MSG_SUID_RESULT);
		if (ret) {
			dev_info(cr->dev, "sns registration: no ack\n");
			goto retry;
		}
		cr->registered = true;
		dev_info(cr->dev, "sns client registered\n");
		schedule_delayed_work(&cr->session_work, 0);
	}
	return;

retry:
	schedule_delayed_work(&cr->lookup_work,
			      msecs_to_jiffies(LOOKUP_RETRY_MS));
}

static bool camres_parse_ctrl(struct oplus_camres *cr, const u8 *buf,
			      size_t len)
{
	u32 cmd, node, port, service;

	if (len < 20)
		return false;
	cmd = get_unaligned_le32(buf);
	if (cmd != QRTR_TYPE_NEW_SERVER && cmd != QRTR_TYPE_DEL_SERVER)
		return false;

	service = get_unaligned_le32(buf + 4);
	/* instance (buf+8) ignored: wildcard, as in libqrtr */
	node = get_unaligned_le32(buf + 12);
	port = get_unaligned_le32(buf + 16);

	if (cmd == QRTR_TYPE_NEW_SERVER) {
		if (!node && !port && !service)
			return true;		/* end of replay */
		if (service == SNS_SERVICE_ID && !cr->sns_found) {
			cr->sns_sq.sq_family = AF_QIPCRTR;
			cr->sns_sq.sq_node = node;
			cr->sns_sq.sq_port = port;
			cr->sns_found = true;
			dev_info(cr->dev, "sns service at %x:%x\n", node, port);
		}
	} else if (service == SNS_SERVICE_ID && cr->sns_found &&
		   cr->sns_sq.sq_node == node && cr->sns_sq.sq_port == port) {
		bool was_ready = cr->session_ready;

		dev_info(cr->dev, "SEE service restart\n");
		cr->sns_found = false;
		cr->registered = false;		/* SEE restart drops it */
		cr->hs_ready = false;
		cr->hs_ok = false;
		cr->aon_armed = false;
		cr->session_ready = false;
		if (was_ready)
			kobject_uevent_env(&cr->dev->kobj, KOBJ_CHANGE,
					   (char *[]){ "CAMRES_READY=0", NULL });
		schedule_delayed_work(&cr->lookup_work, 0);
	}
	return true;
}

/*
 * Attach to SEE's camera session.  Returns 0 only when SEE acked with
 * error == 0 (the session exists and the camera power domain is up).
 */
static int sns_attach(struct oplus_camres *cr)
{
	u8 body[32];
	size_t len;
	int ret;

	if (!cr->registered)
		return -ENOTCONN;

	len = sns_body_init(body, cr->attach_name, cr->attach_slot);
	ret = sns_xfer(cr, cr->suid_lo, cr->suid_hi, QSH_INIT, body, len,
		       QSH_INIT_ACK);
	if (ret)
		return ret;
	if (cr->ack_error) {
		dev_info(cr->dev, "sns attach rejected: error=%u\n",
			 cr->ack_error);
		return -EPROTO;
	}
	dev_info(cr->dev, "sns attach ok (state=%u)\n", cr->ack_state);
	return 0;
}

static int sns_simple(struct oplus_camres *cr, u32 msg, u32 ack,
		      const u8 *body, size_t len)
{
	int ret;

	if (!cr->registered)
		return -ENOTCONN;

	ret = sns_xfer(cr, cr->suid_lo, cr->suid_hi, msg, body, len, ack);
	if (ret)
		return ret;
	if (cr->ack_error) {
		dev_info(cr->dev, "sns 0x%x rejected: error=%u\n",
			 msg, cr->ack_error);
		return -EPROTO;
	}
	return 0;
}

/*
 * Enable the AON camera sensor on SEE (vendor-captured sequence): this
 * is what powers the camera module, once, at session bring-up.  The
 * subscribe is kept standing for the whole session — its ACK (0x20D,
 * error==0) is also the precondition for REVOKE, and after a successful
 * REVOKE the SEE side no longer accepts AON requests.  The sensor
 * streams asynchronously via report indications we do not consume.
 */
/*
 * AON camera config body {1: res_w=320, 2: res_h=240, 3: fps=5,
 * 4: period_ms=1000, 5..7: 0,0,5} — the varint encodings of the fixed
 * values are part of the fixed wire layout.
 */
struct camres_sns_aon_cfg {
	u8 res_w_tag;	u8 res_w[2];
	u8 res_h_tag;	u8 res_h[2];
	u8 fps_tag;	u8 fps;
	u8 period_tag;	u8 period[2];
	u8 f5_tag;	u8 f5;
	u8 f6_tag;	u8 f6;
	u8 f7_tag;	u8 f7;
} __packed;

static int sns_aon_power_on(struct oplus_camres *cr)
{
	static const struct camres_sns_aon_cfg cfg = {
		.res_w_tag = PB_VARINT(1),	.res_w = { 0xc0, 0x02 },
		.res_h_tag = PB_VARINT(2),	.res_h = { 0xf0, 0x01 },
		.fps_tag = PB_VARINT(3),	.fps = 5,
		.period_tag = PB_VARINT(4),	.period = { 0xe8, 0x07 },
		.f5_tag = PB_VARINT(5),		.f5 = 0,
		.f6_tag = PB_VARINT(6),		.f6 = 0,
		.f7_tag = PB_VARINT(7),		.f7 = 5,
	};
	int ret;

	/* attributes request first; reply is an asynchronous event */
	ret = sns_send(cr, SNS_AON_UID_LO, SNS_AON_UID_HI,
		       SNS_MSG_SENSOR_INFO, NULL, 0);
	if (ret)
		return ret;

	return sns_xfer(cr, SNS_AON_UID_LO, SNS_AON_UID_HI,
			SNS_MSG_SENSOR_CONFIG, (const u8 *)&cfg, sizeof(cfg),
			SNS_MSG_SENSOR_CONFIG_ACK);
}

/* tell SEE the AP is done with the camera (power off) */
static void sns_release_notify(struct oplus_camres *cr)
{
	static const struct camres_sns_body_release body = {
		.power_tag = PB_VARINT(1),
		.power = 0,
	};

	if (sns_simple(cr, QSH_RELEASE, QSH_RELEASE_ACK,
		       (const u8 *)&body, sizeof(body)))
		dev_warn(cr->dev, "SEE release notify failed\n");
}

/* ---- SEE camera session readiness ---- */

static int takeover_locked(struct oplus_camres *cr, bool negotiate);

static int camres_reprobe_child(struct device *dev, void *data)
{
	int ret;

	if (dev->driver)
		return 0;

	ret = device_attach(dev);
	dev_dbg(dev, "camres reprobe: %d\n", ret);
	return 0;
}

/*
 * Called without cr->lock: the children's probes acquire the bus and
 * take the lock themselves.
 */
static void camres_session_up(struct oplus_camres *cr)
{
	dev_info(cr->dev, "SEE camera session up\n");
	kobject_uevent_env(&cr->dev->kobj, KOBJ_CHANGE,
			   (char *[]){ "CAMRES_READY=1", NULL });
	/* consumers that deferred on !is_ready() get no other trigger */
	device_for_each_child(&cr->adapter.dev, NULL, camres_reprobe_child);
}

/*
 * SNS registration only proves the ADSP is up.  SEE does not answer the
 * camera handshake until adsprpcd has served its camera config, so poll
 * here until it does:
 *
 *   - INIT without any ACK: adsprpcd is not serving yet — retry.
 *   - INIT ACKed with an error (e.g. the module .pb is missing): the
 *     service is alive — continue; vendor tolerates this too.
 *   - AON subscribe acked: the module is powered and REVOKE is armed.
 *
 * Each step is retried on its own; completed steps are not redone.
 */
static void session_work_fn(struct work_struct *work)
{
	struct oplus_camres *cr = container_of(to_delayed_work(work),
					       struct oplus_camres,
					       session_work);
	int ret;

	mutex_lock(&cr->lock);
	if (cr->session_ready)
		goto out_unlock;

	if (cr->ap_active) {
		/*
		 * AP holds the bus: SEE would reject the AON subscribe
		 * ("camera is acquired").  release_locked re-arms the
		 * session once the bus is handed back — do not claim
		 * readiness without the subscribe standing.
		 */
		goto out_unlock;
	}

	if (!cr->registered)
		goto out_unlock;	/* lookup_work reschedules us */

	if (!cr->hs_ready) {
		ret = sns_attach(cr);
		if (ret == -ETIMEDOUT) {
			mutex_unlock(&cr->lock);
			dev_info(cr->dev,
				 "SEE camera session not up yet (%d)\n",
				 ret);
			schedule_delayed_work(&cr->session_work,
					      msecs_to_jiffies(SESSION_RETRY_MS));
			return;
		}
		if (!ret) {
			sns_release_notify(cr);
			cr->hs_ok = true;
		} else {
			/*
			 * ACKed with an error: the service answered,
			 * only its config is short (no module .pb) —
			 * keep going, but without a valid session
			 * REVOKE/RELEASE must not be sent.
			 */
			dev_warn(cr->dev,
				 "SEE attach rejected (%d): continuing\n",
				 ret);
		}
		cr->hs_ready = true;
	}

	ret = sns_aon_power_on(cr);
	if (ret) {
		mutex_unlock(&cr->lock);
		dev_info(cr->dev,
			 "SEE AON subscribe not up yet (%d)\n", ret);
		schedule_delayed_work(&cr->session_work,
				      msecs_to_jiffies(SESSION_RETRY_MS));
		return;
	}
	cr->aon_armed = true;
	cr->session_ready = true;
	if (!cr->hs_ok) {
		/*
		 * Classified as no-PB (INIT answered with an error):
		 * SEE arbitration is impossible — REVOKE/RELEASE have no
		 * session to act on.  The AON subscribe above is the only
		 * SEE dependency left (it powers the module) and stands;
		 * take the bus once, hold it for good, and let
		 * acquire/release reduce to no-ops (release_work checks
		 * the same hs_ready && !hs_ok condition).
		 */
		dev_info(cr->dev, "no SEE handshake: camera bus held to AP\n");
		ret = takeover_locked(cr, false);
		if (ret)
			dev_warn(cr->dev, "AP hold failed (%d)\n", ret);
	}
	mutex_unlock(&cr->lock);

	camres_session_up(cr);
	return;

out_unlock:
	mutex_unlock(&cr->lock);
}

/* ---- dynamic geni_i2c instantiation ---- */

static int geni_adap_match(struct device *dev, void *data)
{
	struct oplus_camres *cr = data;

	if (dev->of_node == cr->geni_np && dev->type == &i2c_adapter_type) {
		cr->geni_adap = to_i2c_adapter(dev);
		return 1;
	}
	return 0;
}

static int geni_wait_adapter(struct oplus_camres *cr)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(GENI_TIMEOUT_MS);

	do {
		cr->geni_adap = NULL;
		if (bus_for_each_dev(&i2c_bus_type, NULL, cr, geni_adap_match))
			return 0;
		msleep(20);
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

static int geni_create(struct oplus_camres *cr)
{
	struct device_node *np = cr->geni_np;
	struct platform_device *wrapper, *pdev;
	struct resource res[2];
	int ret;

	if (cr->geni_pdev)
		goto wait;

	wrapper = of_find_device_by_node(np->parent);
	if (!wrapper)
		return -ENODEV;

	pdev = platform_device_alloc("camres-i2c", PLATFORM_DEVID_AUTO);
	if (!pdev) {
		platform_device_put(wrapper);
		return -ENOMEM;
	}

	memset(res, 0, sizeof(res));
	ret = of_address_to_resource(np, 0, &res[0]);
	if (ret)
		goto err;
	of_irq_to_resource(np, 0, &res[1]);

	pdev->dev.parent = &wrapper->dev;
	pdev->dev.of_node = of_node_get(np);
	platform_device_add_resources(pdev, res, ARRAY_SIZE(res));

	ret = platform_device_add(pdev);
	platform_device_put(wrapper);
	if (ret)
		goto err;
	cr->geni_pdev = pdev;

wait:
	ret = geni_wait_adapter(cr);
	if (ret)
		return ret;
	cr->geni_dev = &cr->geni_pdev->dev;
	return 0;

err:
	/* platform_device_put releases dev.of_node (the get above) */
	platform_device_put(pdev);
	return ret;
}

/*
 * pm_runtime_disable() before unregister is load-bearing: the driver
 * core resumes the device while unbinding, and its trailing put only
 * queues an autosuspend that can never run on a dying device — the SE
 * clock prepare count would leak.  With pm disabled the core's resume
 * fails without touching hardware, so the device is freed suspended.
 */
static void geni_destroy(struct oplus_camres *cr)
{
	if (!cr->geni_pdev)
		return;

	pm_runtime_disable(&cr->geni_pdev->dev);
	/* unregistering puts dev.of_node (the ref taken in geni_create) */
	platform_device_unregister(cr->geni_pdev);
	cr->geni_pdev = NULL;
	cr->geni_adap = NULL;
	cr->geni_dev = NULL;
}

/* ---- bus ownership ---- */

static void camres_uevent(struct oplus_camres *cr, const char *owner)
{
	char env[16];
	char *envp[] = { env, NULL };

	snprintf(env, sizeof(env), "OWNERSHIP=%s", owner);
	kobject_uevent_env(&cr->dev->kobj, KOBJ_CHANGE, envp);
}

/* called with cr->lock held; @negotiate: run the SEE handshake first */
static int takeover_locked(struct oplus_camres *cr, bool negotiate)
{
	int ret;

	if (cr->ap_active)
		return 0;

	if (negotiate) {
		if (!cr->session_ready) {
			dev_warn(cr->dev, "SEE session not ready\n");
			return -ENOTCONN;
		}
		/*
		 * The AON subscribe that powers the module happened at
		 * session bring-up and stays standing; REVOKE is only
		 * answerable while it does, and only from a valid
		 * handshake session — an out-of-session REVOKE takes the
		 * SEE service down with it.  AON requests die after a
		 * successful REVOKE, so there is nothing to (re)enable.
		 */
		if (revoke) {
			if (!cr->aon_armed) {
				dev_info(cr->dev,
					 "SEE revoke skipped: AON not subscribed\n");
			} else if (!cr->hs_ok) {
				dev_info(cr->dev,
					 "SEE revoke skipped: no handshake session\n");
			} else {
				ret = sns_simple(cr, QSH_REVOKE,
						 QSH_REVOKE_ACK, NULL, 0);
				if (ret)
					dev_warn(cr->dev,
						 "SEE revoke failed (%d)\n",
						 ret);
			}
		}
	}

	ret = pinctrl_select_state(cr->pctl, cr->p_active);
	if (ret)
		return ret;

	ret = geni_create(cr);
	if (ret)
		goto err_pins;

	ret = pm_runtime_resume_and_get(cr->geni_dev);
	if (ret)
		goto err_geni;

	cr->ap_active = true;
	dev_info(cr->dev, "camera bus acquired by AP\n");
	camres_uevent(cr, "AP");
	if (!cr->session_ready)
		schedule_delayed_work(&cr->session_work, 0);
	return 0;

err_geni:
	geni_destroy(cr);
err_pins:
	pinctrl_select_state(cr->pctl, cr->p_released);
	return ret;
}

/* called with cr->lock held; -EAGAIN asks the caller to retry */
static int release_locked(struct oplus_camres *cr)
{
	unsigned long deadline;

	if (!cr->ap_active)
		return 0;

	/* block new transfers on our adapter, then drain in-flight ones */
	WRITE_ONCE(cr->ap_active, false);

	i2c_lock_bus(&cr->adapter, I2C_LOCK_ROOT_ADAPTER);

	pm_runtime_put_sync_suspend(cr->geni_dev);
	deadline = jiffies + msecs_to_jiffies(SUSPEND_TIMEOUT_MS);
	while (!pm_runtime_suspended(cr->geni_dev)) {
		if (time_after(jiffies, deadline)) {
			pm_runtime_resume_and_get(cr->geni_dev);
			i2c_unlock_bus(&cr->adapter, I2C_LOCK_ROOT_ADAPTER);
			WRITE_ONCE(cr->ap_active, true);
			return -EAGAIN;
		}
		msleep(20);
	}

	pinctrl_select_state(cr->pctl, cr->p_released);

	i2c_unlock_bus(&cr->adapter, I2C_LOCK_ROOT_ADAPTER);

	geni_destroy(cr);

	dev_info(cr->dev, "camera bus released to SEE\n");
	if (cr->hs_ok)
		sns_release_notify(cr);
	camres_uevent(cr, "SEE");
	/*
	 * After a SEE restart the AON subscribe (and with it the module
	 * power) is gone; re-arm it now that SEE owns the bus again.
	 */
	if (cr->registered && !cr->aon_armed)
		schedule_delayed_work(&cr->session_work, 0);
	return 0;
}

static void release_work_fn(struct work_struct *work)
{
	struct oplus_camres *cr = container_of(to_delayed_work(work),
					       struct oplus_camres,
					       release_work);

	mutex_lock(&cr->lock);
	/* no-PB (hs_ready && !hs_ok): the bus stays with the AP for good */
	if (!atomic_read(&cr->consumers) && !cr->force && cr->ap_active &&
	    !(cr->hs_ready && !cr->hs_ok)) {
		if (release_locked(cr))
			schedule_delayed_work(&cr->release_work,
					      msecs_to_jiffies(200));
	}
	mutex_unlock(&cr->lock);
}

/* ---- the camres i2c adapter ---- */

static int camres_i2c_xfer(struct i2c_adapter *adap,
			   struct i2c_msg *msgs, int num)
{
	struct oplus_camres *cr = i2c_get_adapdata(adap);

	if (!READ_ONCE(cr->ap_active))
		return -EBUSY;

	return i2c_transfer(cr->geni_adap, msgs, num);
}

static u32 camres_i2c_func(struct i2c_adapter *adap)
{
	struct oplus_camres *cr = i2c_get_adapdata(adap);

	if (cr->geni_adap && cr->geni_adap->algo)
		return cr->geni_adap->algo->functionality(cr->geni_adap);

	/* match geni_i2c's set so client capability checks stay truthful */
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm camres_i2c_algo = {
	.xfer = camres_i2c_xfer,
	.functionality = camres_i2c_func,
};

/* ---- public API ---- */

int oplus_camres_acquire(struct device *consumer)
{
	struct oplus_camres *cr = g_camres;
	int ret;

	if (!cr)
		return -EPROBE_DEFER;

	mutex_lock(&cr->lock);
	ret = takeover_locked(cr, true);
	if (!ret)
		atomic_inc(&cr->consumers);
	mutex_unlock(&cr->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(oplus_camres_acquire);

int oplus_camres_release(struct device *consumer)
{
	struct oplus_camres *cr = g_camres;

	if (!cr)
		return -ENODEV;

	if (atomic_dec_if_positive(&cr->consumers) == 0 && !cr->force)
		schedule_delayed_work(&cr->release_work, 0);
	return 0;
}
EXPORT_SYMBOL_GPL(oplus_camres_release);

bool oplus_camres_ap_holds(void)
{
	return g_camres ? READ_ONCE(g_camres->ap_active) : false;
}
EXPORT_SYMBOL_GPL(oplus_camres_ap_holds);

/*
 * True once the session bring-up completed: SEE answered INIT (an ACK
 * carrying only a config error still counts — the service is up) and
 * the AON subscribe that powers the module was acked.  This needs the
 * ADSP up *and* adsprpcd serving.  Consumers defer their probe while
 * this is false; when it turns true camres reprobes its unbound
 * children and emits a KOBJ_CHANGE uevent (CAMRES_READY=1).
 */
bool oplus_camres_is_ready(void)
{
	struct oplus_camres *cr = g_camres;

	return cr && READ_ONCE(cr->session_ready);
}
EXPORT_SYMBOL_GPL(oplus_camres_is_ready);

/* ---- sysfs ---- */

static int force_set(struct oplus_camres *cr, bool val)
{
	int ret = 0;

	if (val && !cr->force) {
		cr->force = true;
		ret = takeover_locked(cr, true);
		if (ret)
			cr->force = false;
	} else if (!val && cr->force) {
		cr->force = false;
		if (!atomic_read(&cr->consumers))
			schedule_delayed_work(&cr->release_work, 0);
	}
	return ret;
}

static ssize_t force_start_i2c_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct oplus_camres *cr = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", cr->force);
}

static ssize_t force_start_i2c_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct oplus_camres *cr = dev_get_drvdata(dev);
	bool val;
	int ret;

	/* taking the bus from the SEE is a privileged operation */
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&cr->lock);
	ret = force_set(cr, val);
	mutex_unlock(&cr->lock);
	return ret ? ret : count;
}
static struct device_attribute dev_attr_force_start_i2c =
	__ATTR(force_start_i2c, 0600, force_start_i2c_show, force_start_i2c_store);

static ssize_t ap_holds_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", oplus_camres_ap_holds());
}
static DEVICE_ATTR_RO(ap_holds);

static struct attribute *camres_attrs[] = {
	&dev_attr_force_start_i2c.attr,
	&dev_attr_ap_holds.attr,
	NULL,
};
ATTRIBUTE_GROUPS(camres);

/* ---- platform driver ---- */

static int oplus_camres_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct oplus_camres *cr;
	const char *name = NULL;
	int ret;

	cr = devm_kzalloc(dev, sizeof(*cr), GFP_KERNEL);
	if (!cr)
		return -ENOMEM;

	cr->dev = dev;
	cr->suid_hi = 0xa7167528f835f4eeULL;	/* camera_handshake */
	cr->suid_lo = 0xe0e3f77b504b96a1ULL;
	strscpy(cr->attach_name, CAMRES_ATTACH_NAME, sizeof(cr->attach_name));
	cr->attach_slot = CAMRES_ATTACH_SLOT;
	mutex_init(&cr->lock);
	atomic_set(&cr->consumers, 0);
	INIT_DELAYED_WORK(&cr->release_work, release_work_fn);
	INIT_DELAYED_WORK(&cr->lookup_work, lookup_work_fn);
	INIT_DELAYED_WORK(&cr->session_work, session_work_fn);
	INIT_WORK(&cr->rx_work, rx_work_fn);
	init_waitqueue_head(&cr->rx_wq);
	mutex_init(&cr->sns_lock);
	spin_lock_init(&cr->ack_lock);
	init_completion(&cr->ack_done);
	platform_set_drvdata(pdev, cr);

	cr->geni_np = of_parse_phandle(dev->of_node, "i2c-bus", 0);
	if (!cr->geni_np)
		return dev_err_probe(dev, -EINVAL, "missing i2c-bus phandle\n");

	of_property_read_u64(dev->of_node, "sns,suid-hi", &cr->suid_hi);
	of_property_read_u64(dev->of_node, "sns,suid-lo", &cr->suid_lo);
	of_property_read_string(dev->of_node, "sns,attach-name", &name);
	if (name)
		strscpy(cr->attach_name, name, sizeof(cr->attach_name));
	of_property_read_u32(dev->of_node, "sns,attach-slot", &cr->attach_slot);

	/* experiment overrides */
	if (suid_hi)
		cr->suid_hi = suid_hi;
	if (suid_lo)
		cr->suid_lo = suid_lo;
	if (attach_name && attach_name[0])
		strscpy(cr->attach_name, attach_name,
			sizeof(cr->attach_name));
	if (attach_slot >= 0)
		cr->attach_slot = attach_slot;

	cr->pctl = devm_pinctrl_get(dev);
	if (IS_ERR(cr->pctl))
		return dev_err_probe(dev, PTR_ERR(cr->pctl),
				     "failed to get pinctrl\n");

	cr->p_active = pinctrl_lookup_state(cr->pctl, "active");
	if (IS_ERR(cr->p_active))
		return dev_err_probe(dev, PTR_ERR(cr->p_active),
				     "missing active pinctrl state\n");

	cr->p_released = pinctrl_lookup_state(cr->pctl, "released");
	if (IS_ERR(cr->p_released))
		return dev_err_probe(dev, PTR_ERR(cr->p_released),
				     "missing released pinctrl state\n");

	/*
	 * Boot state: the bus belongs to SEE.  Nothing in DT occupies
	 * gpio4/5, so the bootloader's high-Z simply persists — no
	 * pinctrl action here.  States are only selected around
	 * takeovers.
	 */

	cr->adapter.owner = THIS_MODULE;
	cr->adapter.algo = &camres_i2c_algo;
	cr->adapter.dev.parent = dev;
	cr->adapter.dev.of_node = dev->of_node;
	cr->adapter.nr = -1;
	strscpy(cr->adapter.name, "oplus-camres-i2c",
		sizeof(cr->adapter.name));
	i2c_set_adapdata(&cr->adapter, cr);

	ret = i2c_add_adapter(&cr->adapter);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add i2c adapter\n");

	/* resident SNS client: discover + register in the background */
	schedule_delayed_work(&cr->lookup_work, 0);

	g_camres = cr;
	dev_info(dev, "camera bus ready, released to SEE (bus %d)\n",
		 i2c_adapter_id(&cr->adapter));
	return 0;
}

static void oplus_camres_remove(struct platform_device *pdev)
{
	struct oplus_camres *cr = platform_get_drvdata(pdev);

	g_camres = NULL;
	cancel_delayed_work_sync(&cr->release_work);
	cancel_delayed_work_sync(&cr->lookup_work);
	cancel_delayed_work_sync(&cr->session_work);

	mutex_lock(&cr->lock);
	if (cr->ap_active) {
		i2c_lock_bus(&cr->adapter, I2C_LOCK_ROOT_ADAPTER);
		pm_runtime_put_sync_suspend(cr->geni_dev);
		pinctrl_select_state(cr->pctl, cr->p_released);
		i2c_unlock_bus(&cr->adapter, I2C_LOCK_ROOT_ADAPTER);
		geni_destroy(cr);
		cr->ap_active = false;
	}
	mutex_unlock(&cr->lock);

	i2c_del_adapter(&cr->adapter);
	sns_socket_close(cr);
	of_node_put(cr->geni_np);
}

static const struct of_device_id oplus_camres_of_match[] = {
	{ .compatible = "oplus,camres" },
	{ }
};
MODULE_DEVICE_TABLE(of, oplus_camres_of_match);

static struct platform_driver oplus_camres_driver = {
	.probe = oplus_camres_probe,
	.remove = oplus_camres_remove,
	.driver = {
		.name = "oplus-camres",
		.of_match_table = oplus_camres_of_match,
		.dev_groups = camres_groups,
	},
};
module_platform_driver(oplus_camres_driver);

MODULE_AUTHOR("Iceland mainline");
MODULE_DESCRIPTION("OPlus camera shared I2C bus between AP and SEE");
MODULE_LICENSE("GPL");
