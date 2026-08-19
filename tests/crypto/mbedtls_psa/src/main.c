/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Test psa_crypto_init() and psa_generate_random() on the PSA implementation
 * provided by Mbed TLS (platforms using TFM are filtered out in the yaml file).
 */

#include <zephyr/ztest.h>
#include <zephyr/timing/timing.h>

#include <psa/crypto.h>

ZTEST_USER(test_mbedtls_psa, test_generate_random)
{
	uint8_t tmp[64];
	psa_status_t status;

	status = psa_generate_random(tmp, sizeof(tmp));
	zassert_equal(status, PSA_SUCCESS);
}

ZTEST_USER(test_mbedtls_psa, test_sha1)
{
	uint8_t in_buf[] = {'a'};
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_1)] = {0};
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_1)] = {
		0x86, 0xf7, 0xe4, 0x37, 0xfa, 0xa5, 0xa7, 0xfc, 0xe1, 0x5d,
		0x1d, 0xdc, 0xb9, 0xea, 0xea, 0xea, 0x37, 0x76, 0x67, 0xb8};
	size_t out_len;
	psa_status_t status;

	status = psa_hash_compute(PSA_ALG_SHA_1, in_buf, sizeof(in_buf), out_buf, sizeof(out_buf),
				  &out_len);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));
}

ZTEST_USER(test_mbedtls_psa, test_sha224)
{
	uint8_t in_buf[] = {'a'};
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_224)] = {0};
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_224)] = {
		0xab, 0xd3, 0x75, 0x34, 0xc7, 0xd9, 0xa2, 0xef, 0xb9, 0x46, 0x5d, 0xe9, 0x31, 0xcd,
		0x70, 0x55, 0xff, 0xdb, 0x88, 0x79, 0x56, 0x3a, 0xe9, 0x80, 0x78, 0xd6, 0xd6, 0xd5};
	size_t out_len;
	psa_status_t status;
	timing_t start;
	timing_t end;
	uint64_t cycles;
	uint64_t nanoseconds;

	timing_init();
	timing_start();
	start = timing_counter_get();
	status = psa_hash_compute(PSA_ALG_SHA_224, in_buf, sizeof(in_buf), out_buf, sizeof(out_buf),
				  &out_len);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("SHA-224 compute time: %llu ns (%llu cycles)\n", (unsigned long long)nanoseconds,
		 (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));
}

ZTEST_USER(test_mbedtls_psa, test_sha256)
{
	uint8_t in_buf[] = {'a'};
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {0};
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {
		0xca, 0x97, 0x81, 0x12, 0xca, 0x1b, 0xbd, 0xca, 0xfa, 0xc2, 0x31,
		0xb3, 0x9a, 0x23, 0xdc, 0x4d, 0xa7, 0x86, 0xef, 0xf8, 0x14, 0x7c,
		0x4e, 0x72, 0xb9, 0x80, 0x77, 0x85, 0xaf, 0xee, 0x48, 0xbb};
	timing_t start;
	timing_t end;
	uint64_t cycles;
	uint64_t nanoseconds;
	size_t out_len;
	psa_status_t status;

	timing_init();
	timing_start();
	start = timing_counter_get();
	status = psa_hash_compute(PSA_ALG_SHA_256, in_buf, sizeof(in_buf), out_buf, sizeof(out_buf),
				  &out_len);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("SHA-256 compute time: %llu ns (%llu cycles)\n", (unsigned long long)nanoseconds,
		 (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));
}

ZTEST_USER(test_mbedtls_psa, test_sha384)
{
	uint8_t in_buf[] = {'a'};
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_384)] = {0};
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_384)] = {
		0x54, 0xa5, 0x9b, 0x9f, 0x22, 0xb0, 0xb8, 0x08, 0x80, 0xd8, 0x42, 0x7e,
		0x54, 0x8b, 0x7c, 0x23, 0xab, 0xd8, 0x73, 0x48, 0x6e, 0x1f, 0x03, 0x5d,
		0xce, 0x9c, 0xd6, 0x97, 0xe8, 0x51, 0x75, 0x03, 0x3c, 0xaa, 0x88, 0xe6,
		0xd5, 0x7b, 0xc3, 0x5e, 0xfa, 0xe0, 0xb5, 0xaf, 0xd3, 0x14, 0x5f, 0x31};
	size_t out_len;
	psa_status_t status;

	status = psa_hash_compute(PSA_ALG_SHA_384, in_buf, sizeof(in_buf), out_buf, sizeof(out_buf),
				  &out_len);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));
}

ZTEST_USER(test_mbedtls_psa, test_sha512)
{
	uint8_t in_buf[] = {'a'};
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_512)] = {0};
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_512)] = {
		0x1f, 0x40, 0xfc, 0x92, 0xda, 0x24, 0x16, 0x94, 0x75, 0x09, 0x79, 0xee, 0x6c,
		0xf5, 0x82, 0xf2, 0xd5, 0xd7, 0xd2, 0x8e, 0x18, 0x33, 0x5d, 0xe0, 0x5a, 0xbc,
		0x54, 0xd0, 0x56, 0x0e, 0x0f, 0x53, 0x02, 0x86, 0x0c, 0x65, 0x2b, 0xf0, 0x8d,
		0x56, 0x02, 0x52, 0xaa, 0x5e, 0x74, 0x21, 0x05, 0x46, 0xf3, 0x69, 0xfb, 0xbb,
		0xce, 0x8c, 0x12, 0xcf, 0xc7, 0x95, 0x7b, 0x26, 0x52, 0xfe, 0x9a, 0x75};
	size_t out_len;
	psa_status_t status;

	status = psa_hash_compute(PSA_ALG_SHA_512, in_buf, sizeof(in_buf), out_buf, sizeof(out_buf),
				  &out_len);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));
}

ZTEST_USER(test_mbedtls_psa, test_hmac_sha256)
{
	uint8_t key[] = {'a'};
	psa_key_attributes_t key_attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key_id = PSA_KEY_ID_NULL;
	uint8_t in_buf[] = {'a'};
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {0};
	uint8_t out_buf_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {
		0x3e, 0xcf, 0x53, 0x88, 0xe2, 0x20, 0xda, 0x9e, 0x0f, 0x91, 0x94,
		0x85, 0xde, 0xb6, 0x76, 0xd8, 0xbe, 0xe3, 0xae, 0xc0, 0x46, 0xa7,
		0x79, 0x35, 0x3b, 0x46, 0x34, 0x18, 0x51, 0x1e, 0xe6, 0x22};
	timing_t start;
	timing_t end;
	uint64_t cycles;
	uint64_t nanoseconds;
	size_t out_len;
	psa_status_t status;

	psa_set_key_type(&key_attr, PSA_KEY_TYPE_HMAC);
	psa_set_key_algorithm(&key_attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));
	psa_set_key_usage_flags(&key_attr, PSA_KEY_USAGE_SIGN_MESSAGE);

	status = psa_import_key(&key_attr, key, sizeof(key), &key_id);
	zassert_equal(status, PSA_SUCCESS);

	timing_init();
	timing_start();
	start = timing_counter_get();
	status = psa_mac_compute(key_id, PSA_ALG_HMAC(PSA_ALG_SHA_256), in_buf, sizeof(in_buf),
				 out_buf, sizeof(out_buf), &out_len);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("HMAC-SHA-256 compute time: %llu ns (%llu cycles)\n",
		 (unsigned long long)nanoseconds, (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));

	status = psa_destroy_key(key_id);
	zassert_equal(status, PSA_SUCCESS);
}

ZTEST_USER(test_mbedtls_psa, test_aes_ecb)
{
	uint8_t key[] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
			 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf};
	psa_key_attributes_t key_attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key_id = PSA_KEY_ID_NULL;
	uint8_t in_buf[PSA_BLOCK_CIPHER_BLOCK_LENGTH(PSA_KEY_TYPE_AES)];
#define AES_ENCRYPTED_OUTPUT_SIZE                                                                  \
	PSA_CIPHER_ENCRYPT_OUTPUT_SIZE(PSA_KEY_TYPE_AES, PSA_ALG_ECB_NO_PADDING, sizeof(in_buf))
	uint8_t out_buf[AES_ENCRYPTED_OUTPUT_SIZE] = {0};
	uint8_t decrypted[sizeof(in_buf)] = {0};
	uint8_t out_buf_ref[AES_ENCRYPTED_OUTPUT_SIZE] = {0xea, 0x5e, 0x61, 0xae, 0x81, 0x67,
							  0xca, 0xa0, 0x58, 0x63, 0x88, 0xeb,
							  0x9a, 0x7c, 0xb7, 0x55};
	timing_t start;
	timing_t end;
	uint64_t cycles;
	uint64_t nanoseconds;
	size_t out_len;
	psa_status_t status;

	memset(in_buf, 0x5, sizeof(in_buf));

	psa_set_key_type(&key_attr, PSA_KEY_TYPE_AES);
	psa_set_key_algorithm(&key_attr, PSA_ALG_ECB_NO_PADDING);
	psa_set_key_usage_flags(&key_attr, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
	status = psa_import_key(&key_attr, key, sizeof(key), &key_id);
	zassert_equal(status, PSA_SUCCESS);

	timing_init();
	timing_start();
	start = timing_counter_get();
	status = psa_cipher_encrypt(key_id, PSA_ALG_ECB_NO_PADDING, in_buf, sizeof(in_buf), out_buf,
				    sizeof(out_buf), &out_len);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("AES-ECB encrypt time: %llu ns (%llu cycles)\n", (unsigned long long)nanoseconds,
		 (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS);

	zassert_mem_equal(out_buf, out_buf_ref, sizeof(out_buf_ref));

	start = timing_counter_get();
	status = psa_cipher_decrypt(key_id, PSA_ALG_ECB_NO_PADDING, out_buf, out_len, decrypted,
				    sizeof(decrypted), &out_len);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("AES-ECB decrypt time: %llu ns (%llu cycles)\n", (unsigned long long)nanoseconds,
		 (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS);
	zassert_mem_equal(decrypted, in_buf, sizeof(in_buf));

	status = psa_destroy_key(key_id);
	zassert_equal(status, PSA_SUCCESS);
}

/*
 * SHA-256 multipart (psa_hash_operation_t) test data.
 * NIST FIPS 180-4 448-bit message test vector:
 *   "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq" (56 bytes)
 *   SHA-256 = 248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1
 */
static const uint8_t sha256_mp_input[] = {
	'a', 'b', 'c', 'd', 'b', 'c', 'd', 'e', 'c', 'd', 'e', 'f', 'd', 'e', 'f', 'g',
	'e', 'f', 'g', 'h', 'f', 'g', 'h', 'i', 'g', 'h', 'i', 'j', 'h', 'i', 'j', 'k',
	'i', 'j', 'k', 'l', 'j', 'k', 'l', 'm', 'k', 'l', 'm', 'n', 'l', 'm', 'n', 'o',
	'm', 'n', 'o', 'p', 'n', 'o', 'p', 'q'};

static const uint8_t sha256_mp_ref[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {
	0x24, 0x8d, 0x6a, 0x61, 0xd2, 0x06, 0x38, 0xb8, 0xe5, 0xc0, 0x26,
	0x93, 0x0c, 0x3e, 0x60, 0x39, 0xa3, 0x3c, 0xe4, 0x59, 0x64, 0xff,
	0x21, 0x67, 0xf6, 0xec, 0xed, 0xd4, 0x19, 0xdb, 0x06, 0xc1};

static timing_t test_mbedtls_psa_suite_start;

static void *test_mbedtls_psa_setup(void)
{
	timing_init();
	timing_start();
	test_mbedtls_psa_suite_start = timing_counter_get();
	return NULL;
}

static void test_mbedtls_psa_teardown(void *data)
{
	timing_t end = timing_counter_get();
	uint64_t cycles = timing_cycles_get(&test_mbedtls_psa_suite_start, &end);
	uint64_t nanoseconds = timing_cycles_to_ns(cycles);

	(void)data;
	TC_PRINT("=== test_mbedtls_psa suite total time: %llu ns (%llu cycles) ===\n",
		 (unsigned long long)nanoseconds, (unsigned long long)cycles);
}

/*
 * Test SHA-256 multipart: psa_hash_setup → psa_hash_update × 3 → psa_hash_finish.
 * Splits the 56-byte NIST test vector into three chunks (16 + 16 + 24 bytes).
 */
ZTEST_USER(test_mbedtls_psa, test_sha256_multipart)
{
	psa_hash_operation_t op = PSA_HASH_OPERATION_INIT;
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {0};
	timing_t start;
	timing_t end;
	uint64_t cycles;
	uint64_t nanoseconds;
	size_t out_len;
	psa_status_t status;

	timing_init();
	timing_start();
	start = timing_counter_get();

	status = psa_hash_setup(&op, PSA_ALG_SHA_256);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_setup failed (%d)", status);

	/* Chunk 1: bytes  0–15 */
	status = psa_hash_update(&op, sha256_mp_input, 16);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_update (1) failed (%d)", status);
	/* Chunk 2: bytes 16–31 */
	status = psa_hash_update(&op, sha256_mp_input + 16, 16);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_update (2) failed (%d)", status);
	/* Chunk 3: bytes 32–55 */
	status = psa_hash_update(&op, sha256_mp_input + 32, 24);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_update (3) failed (%d)", status);

	status = psa_hash_finish(&op, out_buf, sizeof(out_buf), &out_len);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("SHA-256 multipart (setup+update*3+finish) time: %llu ns (%llu cycles)\n",
		 (unsigned long long)nanoseconds, (unsigned long long)cycles);

	zassert_equal(status, PSA_SUCCESS, "psa_hash_finish failed (%d)", status);
	zassert_equal(out_len, PSA_HASH_LENGTH(PSA_ALG_SHA_256));
	zassert_mem_equal(out_buf, sha256_mp_ref, sizeof(sha256_mp_ref));
}

/*
 * Test SHA-256 multipart: psa_hash_setup → psa_hash_update × 2 → psa_hash_verify.
 * Splits the same 56-byte NIST test vector into two equal 28-byte chunks and
 * verifies the digest in-place without exposing the hash bytes to the caller.
 */
ZTEST_USER(test_mbedtls_psa, test_sha256_multipart_verify)
{
	psa_hash_operation_t op = PSA_HASH_OPERATION_INIT;
	timing_t start;
	timing_t end;
	uint64_t cycles;
	uint64_t nanoseconds;
	psa_status_t status;

	timing_init();
	timing_start();
	start = timing_counter_get();

	status = psa_hash_setup(&op, PSA_ALG_SHA_256);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_setup failed (%d)", status);

	/* Chunk 1: bytes  0–27 */
	status = psa_hash_update(&op, sha256_mp_input, 28);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_update (1) failed (%d)", status);
	/* Chunk 2: bytes 28–55 */
	status = psa_hash_update(&op, sha256_mp_input + 28, 28);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_update (2) failed (%d)", status);

	status = psa_hash_verify(&op, sha256_mp_ref, sizeof(sha256_mp_ref));
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("SHA-256 multipart verify (setup+update*2+verify) time: %llu ns (%llu cycles)\n",
		 (unsigned long long)nanoseconds, (unsigned long long)cycles);

	zassert_equal(status, PSA_SUCCESS, "psa_hash_verify failed (%d)", status);
}

/*
 * Test SHA-256 multipart with psa_hash_clone: feeds the first 32 bytes, clones
 * the mid-stream operation, then feeds the remaining 24 bytes independently to
 * both; both must yield the same final digest.
 */
ZTEST_USER(test_mbedtls_psa, test_sha256_multipart_clone)
{
	psa_hash_operation_t op = PSA_HASH_OPERATION_INIT;
	psa_hash_operation_t op_clone = PSA_HASH_OPERATION_INIT;
	uint8_t out_buf[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {0};
	uint8_t out_clone[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {0};
	timing_t start;
	timing_t end;
	uint64_t cycles;
	uint64_t nanoseconds;
	size_t out_len;
	size_t out_clone_len;
	psa_status_t status;

	timing_init();
	timing_start();
	start = timing_counter_get();

	status = psa_hash_setup(&op, PSA_ALG_SHA_256);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_setup failed (%d)", status);

	/* Feed the first 32 bytes before cloning */
	status = psa_hash_update(&op, sha256_mp_input, 32);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_update failed (%d)", status);

	/* Snapshot the mid-stream state */
	status = psa_hash_clone(&op, &op_clone);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_clone failed (%d)", status);

	/* Feed the remaining 24 bytes to both original and clone independently */
	status = psa_hash_update(&op, sha256_mp_input + 32, 24);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_update (orig) failed (%d)", status);
	status = psa_hash_update(&op_clone, sha256_mp_input + 32, 24);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_update (clone) failed (%d)", status);

	/* Finalise both */
	status = psa_hash_finish(&op, out_buf, sizeof(out_buf), &out_len);
	zassert_equal(status, PSA_SUCCESS, "psa_hash_finish (orig) failed (%d)", status);
	status = psa_hash_finish(&op_clone, out_clone, sizeof(out_clone), &out_clone_len);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("SHA-256 multipart clone (setup+update+clone+update*2+finish*2)"
		 " time: %llu ns (%llu cycles)\n",
		 (unsigned long long)nanoseconds, (unsigned long long)cycles);

	zassert_equal(status, PSA_SUCCESS, "psa_hash_finish (clone) failed (%d)", status);
	zassert_equal(out_len, PSA_HASH_LENGTH(PSA_ALG_SHA_256));
	zassert_equal(out_clone_len, PSA_HASH_LENGTH(PSA_ALG_SHA_256));
	/* Both paths must match the known reference */
	zassert_mem_equal(out_buf, sha256_mp_ref, sizeof(sha256_mp_ref));
	zassert_mem_equal(out_clone, sha256_mp_ref, sizeof(sha256_mp_ref));
	/* And each other */
	zassert_mem_equal(out_buf, out_clone, sizeof(out_buf));
}

ZTEST_SUITE(test_mbedtls_psa, NULL, test_mbedtls_psa_setup, NULL, NULL,
	    test_mbedtls_psa_teardown);
static const uint8_t aes_mode_key[] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
				       0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};

static const uint8_t aes_mode_iv[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
				      0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

static const uint8_t aes_ctr_iv[] = {0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
				     0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff};

static const uint8_t aes_mode_plaintext[] = {
	0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73,
	0x93, 0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7,
	0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4,
	0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef, 0xf6, 0x9f, 0x24, 0x45,
	0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10};

static void test_aes_cipher_mode(const char *mode_name, psa_algorithm_t algorithm,
				 const uint8_t iv[16], const uint8_t *ciphertext_ref,
				 size_t ciphertext_length)
{
	psa_key_attributes_t key_attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_cipher_operation_t operation = PSA_CIPHER_OPERATION_INIT;
	psa_key_id_t key_id = PSA_KEY_ID_NULL;
	uint8_t ciphertext[PSA_CIPHER_ENCRYPT_OUTPUT_SIZE(PSA_KEY_TYPE_AES, PSA_ALG_CBC_PKCS7,
							  sizeof(aes_mode_plaintext))] = {0};
	uint8_t decrypted[sizeof(ciphertext)] = {0};
	timing_t start;
	timing_t end;
	uint64_t cycles;
	uint64_t nanoseconds;
	size_t update_length;
	size_t finish_length;
	size_t output_length;
	psa_status_t status;

	psa_set_key_type(&key_attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&key_attr, PSA_BYTES_TO_BITS(sizeof(aes_mode_key)));
	psa_set_key_algorithm(&key_attr, algorithm);
	psa_set_key_usage_flags(&key_attr, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);

	status = psa_import_key(&key_attr, aes_mode_key, sizeof(aes_mode_key), &key_id);
	zassert_equal(status, PSA_SUCCESS);

	status = psa_cipher_encrypt_setup(&operation, key_id, algorithm);
	zassert_equal(status, PSA_SUCCESS);
	status = psa_cipher_set_iv(&operation, iv, 16);
	zassert_equal(status, PSA_SUCCESS);

	timing_init();
	timing_start();
	start = timing_counter_get();
	status = psa_cipher_update(&operation, aes_mode_plaintext, sizeof(aes_mode_plaintext),
				   ciphertext, sizeof(ciphertext), &update_length);
	finish_length = 0U;
	if (status == PSA_SUCCESS) {
		status = psa_cipher_finish(&operation, ciphertext + update_length,
					   sizeof(ciphertext) - update_length, &finish_length);
	}
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("AES-%s encrypt time: %llu ns (%llu cycles)\n", mode_name,
		 (unsigned long long)nanoseconds, (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS);
	output_length = update_length + finish_length;
	zassert_equal(output_length, ciphertext_length);
	zassert_mem_equal(ciphertext, ciphertext_ref, ciphertext_length);

	operation = psa_cipher_operation_init();
	status = psa_cipher_decrypt_setup(&operation, key_id, algorithm);
	zassert_equal(status, PSA_SUCCESS);
	status = psa_cipher_set_iv(&operation, iv, 16);
	zassert_equal(status, PSA_SUCCESS);

	start = timing_counter_get();
	status = psa_cipher_update(&operation, ciphertext, ciphertext_length, decrypted,
				   sizeof(decrypted), &update_length);
	finish_length = 0U;
	if (status == PSA_SUCCESS) {
		status = psa_cipher_finish(&operation, decrypted + update_length,
					   sizeof(decrypted) - update_length, &finish_length);
	}
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("AES-%s decrypt time: %llu ns (%llu cycles)\n", mode_name,
		 (unsigned long long)nanoseconds, (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS);
	output_length = update_length + finish_length;
	zassert_equal(output_length, sizeof(aes_mode_plaintext));
	zassert_mem_equal(decrypted, aes_mode_plaintext, sizeof(aes_mode_plaintext));

	status = psa_destroy_key(key_id);
	zassert_equal(status, PSA_SUCCESS);
}

ZTEST_USER(psa_crypto_test_suite, test_aes_cbc_no_padding)
{
	static const uint8_t ciphertext_ref[] = {
		0x76, 0x49, 0xab, 0xac, 0x81, 0x19, 0xb2, 0x46, 0xce, 0xe9, 0x8e, 0x9b, 0x12,
		0xe9, 0x19, 0x7d, 0x50, 0x86, 0xcb, 0x9b, 0x50, 0x72, 0x19, 0xee, 0x95, 0xdb,
		0x11, 0x3a, 0x91, 0x76, 0x78, 0xb2, 0x73, 0xbe, 0xd6, 0xb8, 0xe3, 0xc1, 0x74,
		0x3b, 0x71, 0x16, 0xe6, 0x9e, 0x22, 0x22, 0x95, 0x16, 0x3f, 0xf1, 0xca, 0xa1,
		0x68, 0x1f, 0xac, 0x09, 0x12, 0x0e, 0xca, 0x30, 0x75, 0x86, 0xe1, 0xa7};

	test_aes_cipher_mode("CBC-NO-PADDING", PSA_ALG_CBC_NO_PADDING, aes_mode_iv, ciphertext_ref,
			     sizeof(ciphertext_ref));
}

ZTEST_USER(psa_crypto_test_suite, test_aes_cbc_pkcs7)
{
	static const uint8_t ciphertext_ref[] = {
		0x76, 0x49, 0xab, 0xac, 0x81, 0x19, 0xb2, 0x46, 0xce, 0xe9, 0x8e, 0x9b, 0x12, 0xe9,
		0x19, 0x7d, 0x50, 0x86, 0xcb, 0x9b, 0x50, 0x72, 0x19, 0xee, 0x95, 0xdb, 0x11, 0x3a,
		0x91, 0x76, 0x78, 0xb2, 0x73, 0xbe, 0xd6, 0xb8, 0xe3, 0xc1, 0x74, 0x3b, 0x71, 0x16,
		0xe6, 0x9e, 0x22, 0x22, 0x95, 0x16, 0x3f, 0xf1, 0xca, 0xa1, 0x68, 0x1f, 0xac, 0x09,
		0x12, 0x0e, 0xca, 0x30, 0x75, 0x86, 0xe1, 0xa7, 0x8c, 0xb8, 0x28, 0x07, 0x23, 0x0e,
		0x13, 0x21, 0xd3, 0xfa, 0xe0, 0x0d, 0x18, 0xcc, 0x20, 0x12};

	test_aes_cipher_mode("CBC-PKCS7", PSA_ALG_CBC_PKCS7, aes_mode_iv, ciphertext_ref,
			     sizeof(ciphertext_ref));
}

ZTEST_USER(psa_crypto_test_suite, test_aes_cfb)
{
	static const uint8_t ciphertext_ref[] = {
		0x3b, 0x3f, 0xd9, 0x2e, 0xb7, 0x2d, 0xad, 0x20, 0x33, 0x34, 0x49, 0xf8, 0xe8,
		0x3c, 0xfb, 0x4a, 0xc8, 0xa6, 0x45, 0x37, 0xa0, 0xb3, 0xa9, 0x3f, 0xcd, 0xe3,
		0xcd, 0xad, 0x9f, 0x1c, 0xe5, 0x8b, 0x26, 0x75, 0x1f, 0x67, 0xa3, 0xcb, 0xb1,
		0x40, 0xb1, 0x80, 0x8c, 0xf1, 0x87, 0xa4, 0xf4, 0xdf, 0xc0, 0x4b, 0x05, 0x35,
		0x7c, 0x5d, 0x1c, 0x0e, 0xea, 0xc4, 0xc6, 0x6f, 0x9f, 0xf7, 0xf2, 0xe6};

	test_aes_cipher_mode("CFB", PSA_ALG_CFB, aes_mode_iv, ciphertext_ref,
			     sizeof(ciphertext_ref));
}

ZTEST_USER(psa_crypto_test_suite, test_aes_ofb)
{
	static const uint8_t ciphertext_ref[] = {
		0x3b, 0x3f, 0xd9, 0x2e, 0xb7, 0x2d, 0xad, 0x20, 0x33, 0x34, 0x49, 0xf8, 0xe8,
		0x3c, 0xfb, 0x4a, 0x77, 0x89, 0x50, 0x8d, 0x16, 0x91, 0x8f, 0x03, 0xf5, 0x3c,
		0x52, 0xda, 0xc5, 0x4e, 0xd8, 0x25, 0x97, 0x40, 0x05, 0x1e, 0x9c, 0x5f, 0xec,
		0xf6, 0x43, 0x44, 0xf7, 0xa8, 0x22, 0x60, 0xed, 0xcc, 0x30, 0x4c, 0x65, 0x28,
		0xf6, 0x59, 0xc7, 0x78, 0x66, 0xa5, 0x10, 0xd9, 0xc1, 0xd6, 0xae, 0x5e};

	test_aes_cipher_mode("OFB", PSA_ALG_OFB, aes_mode_iv, ciphertext_ref,
			     sizeof(ciphertext_ref));
}

ZTEST_USER(psa_crypto_test_suite, test_aes_ctr)
{
	static const uint8_t ciphertext_ref[] = {
		0x87, 0x4d, 0x61, 0x91, 0xb6, 0x20, 0xe3, 0x26, 0x1b, 0xef, 0x68, 0x64, 0x99,
		0x0d, 0xb6, 0xce, 0x98, 0x06, 0xf6, 0x6b, 0x79, 0x70, 0xfd, 0xff, 0x86, 0x17,
		0x18, 0x7b, 0xb9, 0xff, 0xfd, 0xff, 0x5a, 0xe4, 0xdf, 0x3e, 0xdb, 0xd5, 0xd3,
		0x5e, 0x5b, 0x4f, 0x09, 0x02, 0x0d, 0xb0, 0x3e, 0xab, 0x1e, 0x03, 0x1d, 0xda,
		0x2f, 0xbe, 0x03, 0xd1, 0x79, 0x21, 0x70, 0xa0, 0xf3, 0x00, 0x9c, 0xee};

	test_aes_cipher_mode("CTR", PSA_ALG_CTR, aes_ctr_iv, ciphertext_ref,
			     sizeof(ciphertext_ref));
}

ZTEST_USER(psa_crypto_test_suite, test_aes_ccm)
{
	static const uint8_t key[] = {0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
				      0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf};
	static const uint8_t nonce[] = {0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x00,
					0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5};
	static const uint8_t additional_data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	static const uint8_t plaintext[] = {0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
					    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
					    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e};
	static const uint8_t ciphertext_ref[] = {0x58, 0x8c, 0x97, 0x9a, 0x61, 0xc6, 0x63, 0xd2,
						 0xf0, 0x66, 0xd0, 0xc2, 0xc0, 0xf9, 0x89, 0x80,
						 0x6d, 0x5f, 0x6b, 0x61, 0xda, 0xc3, 0x84, 0x17,
						 0xe8, 0xd1, 0x2c, 0xfd, 0xf9, 0x26, 0xe0};
	psa_key_attributes_t key_attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_algorithm_t algorithm = PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, 8);
	psa_key_id_t key_id = PSA_KEY_ID_NULL;
	uint8_t ciphertext[sizeof(ciphertext_ref)] = {0};
	uint8_t decrypted[sizeof(plaintext)] = {0};
	timing_t start;
	timing_t end;
	uint64_t cycles;
	uint64_t nanoseconds;
	size_t output_length;
	psa_status_t status;

	psa_set_key_type(&key_attr, PSA_KEY_TYPE_AES);
	psa_set_key_algorithm(&key_attr, algorithm);
	psa_set_key_usage_flags(&key_attr, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);

	status = psa_import_key(&key_attr, key, sizeof(key), &key_id);
	zassert_equal(status, PSA_SUCCESS);

	timing_init();
	timing_start();
	start = timing_counter_get();
	status = psa_aead_encrypt(key_id, algorithm, nonce, sizeof(nonce), additional_data,
				  sizeof(additional_data), plaintext, sizeof(plaintext), ciphertext,
				  sizeof(ciphertext), &output_length);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("AES-CCM encrypt time: %llu ns (%llu cycles)\n", (unsigned long long)nanoseconds,
		 (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS);
	zassert_equal(output_length, sizeof(ciphertext_ref));
	zassert_mem_equal(ciphertext, ciphertext_ref, sizeof(ciphertext_ref));

	start = timing_counter_get();
	status = psa_aead_decrypt(key_id, algorithm, nonce, sizeof(nonce), additional_data,
				  sizeof(additional_data), ciphertext, sizeof(ciphertext),
				  decrypted, sizeof(decrypted), &output_length);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("AES-CCM decrypt time: %llu ns (%llu cycles)\n", (unsigned long long)nanoseconds,
		 (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS);
	zassert_equal(output_length, sizeof(plaintext));
	zassert_mem_equal(decrypted, plaintext, sizeof(plaintext));

	status = psa_destroy_key(key_id);
	zassert_equal(status, PSA_SUCCESS);
}

ZTEST_USER(psa_crypto_test_suite, test_ecc_ecdsa)
{
	static const uint8_t hash[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {
		0x2c, 0xf2, 0x4d, 0xba, 0x5f, 0xb0, 0xa3, 0x0e, 0x26, 0xe8, 0x3b,
		0x2a, 0xc5, 0xb9, 0xe2, 0x9e, 0x1b, 0x16, 0x1e, 0x5c, 0x1f, 0xa7,
		0x42, 0x5e, 0x73, 0x04, 0x33, 0x62, 0x93, 0x8b, 0x98, 0x24};
	psa_key_attributes_t key_attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_algorithm_t algorithm = PSA_ALG_ECDSA(PSA_ALG_SHA_256);
	psa_key_id_t key_id = PSA_KEY_ID_NULL;
	uint8_t signature[PSA_SIGN_OUTPUT_SIZE(PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1),
					       256, algorithm)];
	timing_t start;
	timing_t end;
	uint64_t cycles;
	uint64_t nanoseconds;
	size_t signature_length;
	psa_status_t status;

	psa_set_key_type(&key_attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
	psa_set_key_bits(&key_attr, 256);
	psa_set_key_usage_flags(&key_attr, PSA_KEY_USAGE_SIGN_HASH | PSA_KEY_USAGE_VERIFY_HASH);
	psa_set_key_algorithm(&key_attr, algorithm);

	status = psa_generate_key(&key_attr, &key_id);
	zassert_equal(status, PSA_SUCCESS, "Unable to generate P-256 key (%d)", status);

	timing_init();
	timing_start();
	start = timing_counter_get();
	status = psa_sign_hash(key_id, algorithm, hash, sizeof(hash), signature, sizeof(signature),
			       &signature_length);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("P-256 ECDSA sign time: %llu ns (%llu cycles)\n", (unsigned long long)nanoseconds,
		 (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS, "ECDSA signing failed (%d)", status);

	start = timing_counter_get();
	status =
		psa_verify_hash(key_id, algorithm, hash, sizeof(hash), signature, signature_length);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("P-256 ECDSA verify time: %llu ns (%llu cycles)\n",
		 (unsigned long long)nanoseconds, (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS, "ECDSA verification failed (%d)", status);

	status = psa_destroy_key(key_id);
	zassert_equal(status, PSA_SUCCESS);
}

ZTEST_USER(psa_crypto_test_suite, test_ecc_ecdh)
{
	psa_key_attributes_t key_attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key_id_a = PSA_KEY_ID_NULL;
	psa_key_id_t key_id_b = PSA_KEY_ID_NULL;
	uint8_t public_key_a[PSA_EXPORT_PUBLIC_KEY_OUTPUT_SIZE(
		PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1), 256)];
	uint8_t public_key_b[sizeof(public_key_a)];
	uint8_t secret_a[32];
	uint8_t secret_b[32];
	timing_t start;
	timing_t end;
	uint64_t cycles;
	uint64_t nanoseconds;
	size_t public_key_a_length;
	size_t public_key_b_length;
	size_t secret_a_length;
	size_t secret_b_length;
	psa_status_t status;

	psa_set_key_type(&key_attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
	psa_set_key_bits(&key_attr, 256);
	psa_set_key_usage_flags(&key_attr, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&key_attr, PSA_ALG_ECDH);

	status = psa_generate_key(&key_attr, &key_id_a);
	zassert_equal(status, PSA_SUCCESS, "Unable to generate key A (%d)", status);
	status = psa_generate_key(&key_attr, &key_id_b);
	zassert_equal(status, PSA_SUCCESS, "Unable to generate key B (%d)", status);

	status = psa_export_public_key(key_id_a, public_key_a, sizeof(public_key_a),
				       &public_key_a_length);
	zassert_equal(status, PSA_SUCCESS);
	status = psa_export_public_key(key_id_b, public_key_b, sizeof(public_key_b),
				       &public_key_b_length);
	zassert_equal(status, PSA_SUCCESS);

	timing_init();
	timing_start();
	start = timing_counter_get();
	status = psa_raw_key_agreement(PSA_ALG_ECDH, key_id_a, public_key_b, public_key_b_length,
				       secret_a, sizeof(secret_a), &secret_a_length);
	end = timing_counter_get();
	cycles = timing_cycles_get(&start, &end);
	nanoseconds = timing_cycles_to_ns(cycles);
	TC_PRINT("P-256 ECDH key agreement time: %llu ns (%llu cycles)\n",
		 (unsigned long long)nanoseconds, (unsigned long long)cycles);
	zassert_equal(status, PSA_SUCCESS, "ECDH key agreement A failed (%d)", status);

	status = psa_raw_key_agreement(PSA_ALG_ECDH, key_id_b, public_key_a, public_key_a_length,
				       secret_b, sizeof(secret_b), &secret_b_length);
	zassert_equal(status, PSA_SUCCESS, "ECDH key agreement B failed (%d)", status);
	zassert_equal(secret_a_length, secret_b_length);
	zassert_mem_equal(secret_a, secret_b, secret_a_length);

	status = psa_destroy_key(key_id_a);
	zassert_equal(status, PSA_SUCCESS);
	status = psa_destroy_key(key_id_b);
	zassert_equal(status, PSA_SUCCESS);
}

ZTEST_SUITE(psa_crypto_test_suite, NULL, NULL, NULL, NULL, NULL);
