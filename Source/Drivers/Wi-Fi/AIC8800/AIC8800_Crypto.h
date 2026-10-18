#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * AIC8800_Crypto.h -- the small set of primitives the in-driver WPA2-PSK
 * supplicant needs, implemented locally because a driver module is
 * freestanding (it cannot link against Library/Crypto/):
 *
 *   SHA-1 and HMAC-SHA1   -- the WPA2-CCMP key hierarchy (PMK/PTK/MIC)
 *   PBKDF2-HMAC-SHA1      -- passphrase -> PMK (4096 iterations)
 *   AES-128 encrypt/decrypt -- for NIST SP 800-38F key unwrap
 *   aes128_kw_unwrap      -- the GTK the AP sends in message 3 of 4 is
 *                            wrapped with the KEK; the AP only ever wraps
 *                            16-byte keys here, but the routine handles
 *                            any nIST-wrapped length.
 *
 * These are straight implementations of FIPS 180-4 / RFC 2104 / RFC
 * 2898 / FIPS 197 -- nothing vendor-specific.
 */

typedef struct {
    uint32_t h[5];
    uint64_t total_len;
    uint8_t  block[64];
    uint32_t block_len;
} aic_sha1_ctx_t;

void aic_sha1_init(aic_sha1_ctx_t *ctx);
void aic_sha1_update(aic_sha1_ctx_t *ctx, const uint8_t *data, uint32_t len);
void aic_sha1_final(aic_sha1_ctx_t *ctx, uint8_t out[20]);

void aic_hmac_sha1(const uint8_t *key, uint32_t key_len,
                   const uint8_t *data, uint32_t data_len,
                   uint8_t out[20]);

void aic_pbkdf2_sha1(const uint8_t *pass, uint32_t pass_len,
                     const uint8_t *salt, uint32_t salt_len,
                     uint32_t iterations,
                     uint8_t *out, uint32_t out_len);

typedef struct {
    uint8_t round_keys[176]; /* 11 x 16 bytes */
} aic_aes128_ctx_t;

void aic_aes128_init(aic_aes128_ctx_t *ctx, const uint8_t key[16]);
void aic_aes128_encrypt(const aic_aes128_ctx_t *ctx, const uint8_t in[16], uint8_t out[16]);
void aic_aes128_decrypt(const aic_aes128_ctx_t *ctx, const uint8_t in[16], uint8_t out[16]);

/* NIST SP 800-38F AES key unwrap ("AES-KW", RFC 3394). Accepts a
 * `wrapped_len` that is a multiple of 8 and >= 24. Returns false when
 * the integrity check fails (wrong KEK / corrupted data). */
bool aic_aes128_kw_unwrap(const uint8_t kek[16],
                          const uint8_t *wrapped, uint32_t wrapped_len,
                          uint8_t *out, uint32_t out_cap, uint32_t *out_len);

/* Best-effort random bytes for the supplicant's SNonce: a xorshift64*
 * generator seeded from the kernel's monotonic clock and the tick
 * counter, re-stirred on every call. Adequate for WPA2 nonce freshness
 * (the security of the handshake rests on the PMK, not on nonce
 * secrecy), and this driver has no CSPRNG to draw from. */
void aic_random_bytes(uint8_t *out, uint32_t len);
