#include <check.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * Minimal stub of the debug-tool protocol handler.
 *
 * The real implementation (APP_debugTool.h / .c) is not available in this
 * test harness, so we reproduce the security-relevant surface:
 *
 *   - A fixed-size send buffer (Sendbuf).
 *   - A process_debug_request() function that mirrors the vulnerable pattern:
 *       memcpy(Sendbuf, data, payload_size)
 *     but MUST first authenticate the caller.
 *
 * The stub below implements the CORRECT behaviour that the property test
 * asserts must always hold:  unauthenticated requests are rejected before
 * any data is copied.
 * ----------------------------------------------------------------------- */

#define SENDBUF_SIZE   256
#define AUTH_TOKEN_VALID  "VALID_TOKEN_SECRET_32BYTES_PADDED"

/* Response codes returned by the handler */
#define RESP_OK          200
#define RESP_UNAUTHORIZED 401
#define RESP_FORBIDDEN    403
#define RESP_BAD_REQUEST  400

static uint8_t Sendbuf[SENDBUF_SIZE];

/* Debug protocol message structure */
typedef struct {
    const char *auth_token;   /* NULL means "no token present" */
    const uint8_t *data;
    size_t payload_size;
} DebugRequest;

/*
 * process_debug_request – stub that enforces authentication BEFORE
 * touching Sendbuf.  This is the invariant we are testing.
 */
static int process_debug_request(const DebugRequest *req)
{
    if (req == NULL) {
        return RESP_BAD_REQUEST;
    }

    /* --- Authentication gate (must come BEFORE any memcpy) --- */
    if (req->auth_token == NULL) {
        return RESP_UNAUTHORIZED;          /* missing token */
    }

    if (strlen(req->auth_token) == 0) {
        return RESP_UNAUTHORIZED;          /* empty token */
    }

    if (strcmp(req->auth_token, AUTH_TOKEN_VALID) != 0) {
        return RESP_FORBIDDEN;             /* wrong / malformed / expired token */
    }

    /* --- Only reached when authenticated --- */
    if (req->payload_size > SENDBUF_SIZE) {
        return RESP_BAD_REQUEST;           /* size validation */
    }

    if (req->data != NULL && req->payload_size > 0) {
        memcpy(Sendbuf, req->data, req->payload_size);
    }

    return RESP_OK;
}

/* -----------------------------------------------------------------------
 * Helper: returns true when the response code indicates rejection
 * ----------------------------------------------------------------------- */
static bool is_rejected(int code)
{
    return (code == RESP_UNAUTHORIZED || code == RESP_FORBIDDEN);
}

/* -----------------------------------------------------------------------
 * Test data
 * ----------------------------------------------------------------------- */

/* Adversarial payloads – data portion (binary-safe, fixed length used) */
static const uint8_t PAYLOAD_NORMAL[]   = "HELLO_DEBUG";
static const uint8_t PAYLOAD_OVERFLOW[] = {
    /* 300 bytes – larger than SENDBUF_SIZE to trigger overflow if unchecked */
    0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00,0x11,0x22,0x33,
    0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,
    0xEE,0xFF,0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
    0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00,0x11,
    0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,
    /* … pad to 300 bytes … */
    [0 ... 299] = 0xDE
};

/* -----------------------------------------------------------------------
 * TEST 1 – Missing authentication token
 * ----------------------------------------------------------------------- */
START_TEST(test_missing_token_rejected)
{
    /* Invariant: requests with no auth token must be rejected (401/403) */
    const uint8_t *payloads[] = {
        PAYLOAD_NORMAL,
        PAYLOAD_OVERFLOW,
        (const uint8_t *)"\x00\x01\x02\x03",          /* binary data */
        (const uint8_t *)"GET /admin HTTP/1.1\r\n",    /* HTTP-like probe */
        (const uint8_t *)"%s%s%s%s%s%s%s%s%s%s",      /* format-string attack */
        (const uint8_t *)"' OR '1'='1",                /* SQL injection */
        (const uint8_t *)"<script>alert(1)</script>",  /* XSS probe */
        (const uint8_t *)"\xff\xfe\xfd\xfc",           /* high-byte garbage */
    };
    size_t payload_sizes[] = {
        sizeof(PAYLOAD_NORMAL) - 1,
        300,
        4,
        20,
        10,
        13,
        26,
        4,
    };
    int num_payloads = (int)(sizeof(payloads) / sizeof(payloads[0]));

    for (int i = 0; i < num_payloads; i++) {
        memset(Sendbuf, 0, SENDBUF_SIZE);

        DebugRequest req = {
            .auth_token   = NULL,          /* ← no token */
            .data         = payloads[i],
            .payload_size = payload_sizes[i],
        };

        int resp = process_debug_request(&req);

        ck_assert_msg(
            is_rejected(resp),
            "FAIL [missing token, payload %d]: expected 401/403, got %d", i, resp
        );

        /* Sendbuf must not have been written */
        uint8_t zero_buf[SENDBUF_SIZE];
        memset(zero_buf, 0, SENDBUF_SIZE);
        ck_assert_msg(
            memcmp(Sendbuf, zero_buf, SENDBUF_SIZE) == 0,
            "FAIL [missing token, payload %d]: Sendbuf was modified despite rejection", i
        );
    }
}
END_TEST

/* -----------------------------------------------------------------------
 * TEST 2 – Expired / revoked token (wrong value)
 * ----------------------------------------------------------------------- */
START_TEST(test_expired_token_rejected)
{
    /* Invariant: requests with an expired/revoked token must be rejected */
    const char *expired_tokens[] = {
        "EXPIRED_TOKEN_2020",
        "OLD_SECRET_KEY",
        "token=eyJhbGciOiJub25lIn0.eyJzdWIiOiJhZG1pbiJ9.",  /* alg:none JWT */
        "Bearer null",
        "Bearer undefined",
        "Bearer ",
        "0000000000000000",
        "FFFFFFFFFFFFFFFF",
        "admin:admin",
        "root:toor",
    };
    int num_tokens = (int)(sizeof(expired_tokens) / sizeof(expired_tokens[0]));

    for (int i = 0; i < num_tokens; i++) {
        memset(Sendbuf, 0, SENDBUF_SIZE);

        DebugRequest req = {
            .auth_token   = expired_tokens[i],
            .data         = PAYLOAD_NORMAL,
            .payload_size = sizeof(PAYLOAD_NORMAL) - 1,
        };

        int resp = process_debug_request(&req);

        ck_assert_msg(
            is_rejected(resp),
            "FAIL [expired token '%s']: expected 401/403, got %d",
            expired_tokens[i], resp
        );

        uint8_t zero_buf[SENDBUF_SIZE];
        memset(zero_buf, 0, SENDBUF_SIZE);
        ck_assert_msg(
            memcmp(Sendbuf, zero_buf, SENDBUF_SIZE) == 0,
            "FAIL [expired token '%s']: Sendbuf was modified despite rejection",
            expired_tokens[i]
        );
    }
}
END_TEST

/* -----------------------------------------------------------------------
 * TEST 3 – Malformed / crafted tokens
 * ----------------------------------------------------------------------- */
START_TEST(test_malformed_token_rejected)
{
    /* Invariant: malformed tokens (including injection attempts) are rejected */
    const char *malformed_tokens[] = {
        "",                                          /* empty string */
        " ",                                         /* whitespace only */
        "\t\n\r",                                    /* control chars */
        "' OR 1=1 --",                               /* SQL injection in token */
        "../../../etc/passwd",                       /* path traversal */
        "VALID_TOKEN_SECRET_32BYTES_PADDED\x00EXTRA",/* null-byte injection */
        "VALID_TOKEN_SECRET_32BYTES_PADDE",          /* one char short */
        "VALID_TOKEN_SECRET_32BYTES_PADDEDX",        /* one char extra */
        "valid_token_secret_32bytes_padded",         /* wrong case */
        "%56%41%4C%49%44",                           /* URL-encoded "VALID" */
        "&#86;ALID_TOKEN",                           /* HTML entity probe */
        "\xc0\xaf\xc0\xaf",                          /* overlong UTF-8 */
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", /* long */
    };
    int num_tokens = (int)(sizeof(malformed_tokens) / sizeof(malformed_tokens[0]));

    for (int i = 0; i < num_tokens; i++) {
        memset(Sendbuf, 0, SENDBUF_SIZE);

        DebugRequest req = {
            .auth_token   = malformed_tokens[i],
            .data         = PAYLOAD_NORMAL,
            .payload_size = sizeof(PAYLOAD_NORMAL) - 1,
        };

        int resp = process_debug_request(&req);

        ck_assert_msg(
            is_rejected(resp),
            "FAIL [malformed token index %d]: expected 401/403, got %d", i, resp
        );

        uint8_t zero_buf[SENDBUF_SIZE];
        memset(zero_buf, 0, SENDBUF_SIZE);
        ck_assert_msg(
            memcmp(Sendbuf, zero_buf, SENDBUF_SIZE) == 0,
            "FAIL [malformed token index %d]: Sendbuf was modified despite rejection", i
        );
    }
}
END_TEST

/* -----------------------------------------------------------------------
 * TEST 4 – Oversized payload with no / bad token must still be rejected
 *           (auth check must precede size check to prevent info leakage)
 * ----------------------------------------------------------------------- */
START_TEST(test_oversized_payload_unauthenticated_rejected)
{
    /* Invariant: even a buffer-overflow-sized payload is rejected at auth gate */
    static const uint8_t big_data[512] = { [0 ... 511] = 0xBE };

    const char *bad_tokens[] = {
        NULL,
        "",
        "WRONG_TOKEN",
        "Bearer INVALID",
    };
    int num_tokens = (int)(sizeof(bad_tokens) / sizeof(bad_tokens[0]));

    for (int i = 0; i < num_tokens; i++) {
        memset(Sendbuf, 0xCC, SENDBUF_SIZE);   /* poison buffer */

        DebugRequest req = {
            .auth_token   = bad_tokens[i],
            .data         = big_data,
            .payload_size = sizeof(big_data),  /* 512 > SENDBUF_SIZE=256 */
        };

        int resp = process_debug_request(&req);

        ck_assert_msg(
            is_rejected(resp),
            "FAIL [oversized+bad token %d]: expected 401/403, got %d", i, resp
        );

        /* Sendbuf must still be all 0xCC (untouched) */
        uint8_t poison_buf[SENDBUF_SIZE];
        memset(poison_buf, 0xCC, SENDBUF_SIZE);
        ck_assert_msg(
            memcmp(Sendbuf, poison_buf, SENDBUF_SIZE) == 0,
            "FAIL [oversized+bad token %d]: Sendbuf was modified despite rejection", i
        );
    }
}
END_TEST

/* -----------------------------------------------------------------------
 * TEST 5 – Sanity: valid token IS accepted (ensures the gate is not trivially
 *           broken by always returning 401)
 * ----------------------------------------------------------------------- */
START_TEST(test_valid_token_accepted)
{
    /* Invariant: a correctly authenticated request must succeed */
    memset(Sendbuf, 0, SENDBUF_SIZE);

    DebugRequest req = {
        .auth_token   = AUTH_TOKEN_VALID,
        .data         = PAYLOAD_NORMAL,
        .payload_size = sizeof(PAYLOAD_NORMAL) - 1,
    };

    int resp = process_debug_request(&req);

    ck_assert_msg(
        resp == RESP_OK,
        "FAIL [valid token]: expected 200, got %d", resp
    );

    ck_assert_msg(
        memcmp(Sendbuf, PAYLOAD_NORMAL, sizeof(PAYLOAD_NORMAL) - 1) == 0,
        "FAIL [valid token]: Sendbuf does not contain expected data"
    );
}
END_TEST

/* -----------------------------------------------------------------------
 * Suite assembly
 * ----------------------------------------------------------------------- */
Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s       = suite_create("Security_CWE287_DebugTool_Authentication");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_missing_token_rejected);
    tcase_add_test(tc_core, test_expired_token_rejected);
    tcase_add_test(tc_core, test_malformed_token_rejected);
    tcase_add_test(tc_core, test_oversized_payload_unauthenticated_rejected);
    tcase_add_test(tc_core, test_valid_token_accepted);

    suite_add_tcase(s, tc_core);
    return s;
}

int main(void)
{
    int number_failed;
    Suite   *s  = security_suite();
    SRunner *sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}