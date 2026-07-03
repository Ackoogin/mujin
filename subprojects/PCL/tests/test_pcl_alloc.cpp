/// \file test_pcl_alloc.cpp
/// \brief Unit tests for the portable heap allocator (pcl_alloc.h).
///
/// Requirements coverage: REQ_PCL_213 .. REQ_PCL_216 (allocator semantics).
#include <gtest/gtest.h>

#include <cstring>

extern "C" {
#include "pcl/pcl_alloc.h"
}

///< REQ_PCL_213: pcl_alloc returns usable memory; zero-size returns NULL.
TEST(PclAlloc, AllocAndFreeRoundTrip) {
  void* p = pcl_alloc(64u);
  ASSERT_NE(p, nullptr);
  memset(p, 0xAB, 64u);
  pcl_free(p);

  EXPECT_EQ(pcl_alloc(0u), nullptr);
}

///< REQ_PCL_214: pcl_calloc zero-initializes; zero counts and overflowing
///< nmemb*size requests are rejected with NULL.
TEST(PclAlloc, CallocZeroesAndRejectsOverflow) {
  unsigned char* p = (unsigned char*)pcl_calloc(16u, 4u);
  ASSERT_NE(p, nullptr);
  for (size_t i = 0; i < 64u; ++i) EXPECT_EQ(p[i], 0u);
  pcl_free(p);

  EXPECT_EQ(pcl_calloc(0u, 8u), nullptr);
  EXPECT_EQ(pcl_calloc(8u, 0u), nullptr);
  // nmemb * size overflows size_t: must fail rather than under-allocate.
  EXPECT_EQ(pcl_calloc((size_t)-1, 2u), nullptr);
}

///< REQ_PCL_215: pcl_realloc grows an allocation preserving contents,
///< acts as pcl_alloc for a NULL pointer, and frees when size is zero.
TEST(PclAlloc, ReallocSemantics) {
  // NULL pointer: behaves as pcl_alloc.
  char* p = (char*)pcl_realloc(nullptr, 8u);
  ASSERT_NE(p, nullptr);
  memcpy(p, "abcdefg", 8u);

  // Grow: contents preserved.
  p = (char*)pcl_realloc(p, 4096u);
  ASSERT_NE(p, nullptr);
  EXPECT_STREQ(p, "abcdefg");

  // Zero size: frees and returns NULL.
  EXPECT_EQ(pcl_realloc(p, 0u), nullptr);
}

///< REQ_PCL_216: pcl_free(NULL) is a safe no-op.
TEST(PclAlloc, FreeNullIsNoOp) {
  pcl_free(nullptr);
}
