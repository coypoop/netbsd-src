/*	$NetBSD: run-packetProcessing.c,v 1.1.1.5 2016/01/08 21:21:33 christos Exp $	*/

/* AUTOGENERATED FILE. DO NOT EDIT. */

//=======Test Runner Used To Run Each Test Below=====
#define RUN_TEST(TestFunc, TestLineNum) \
{ \
  Unity.CurrentTestName = #TestFunc; \
  Unity.CurrentTestLineNumber = TestLineNum; \
  Unity.NumberOfTests++; \
  if (TEST_PROTECT()) \
  { \
      setUp(); \
      TestFunc(); \
  } \
  if (TEST_PROTECT() && !TEST_IS_IGNORED) \
  { \
    tearDown(); \
  } \
  UnityConcludeTest(); \
}

//=======Automagically Detected Files To Include=====
#include "unity.h"
#include <setjmp.h>
#include <stdio.h>
#include "config.h"
#include "sntptest.h"
#include "networking.h"
#include "ntp_stdlib.h"

//=======External Functions This Runner Calls=====
extern void setUp(void);
extern void tearDown(void);
extern void test_TooShortLength(void);
extern void test_LengthNotMultipleOfFour(void);
extern void test_TooShortExtensionFieldLength(void);
extern void test_UnauthenticatedPacketReject(void);
extern void test_CryptoNAKPacketReject(void);
extern void test_AuthenticatedPacketInvalid(void);
extern void test_AuthenticatedPacketUnknownKey(void);
extern void test_ServerVersionTooOld(void);
extern void test_ServerVersionTooNew(void);
extern void test_NonWantedMode(void);
extern void test_KoDRate(void);
extern void test_KoDDeny(void);
extern void test_RejectUnsyncedServer(void);
extern void test_RejectWrongResponseServerMode(void);
extern void test_AcceptNoSentPacketBroadcastMode(void);
extern void test_CorrectUnauthenticatedPacket(void);
extern void test_CorrectAuthenticatedPacketMD5(void);
extern void test_CorrectAuthenticatedPacketSHA1(void);


//=======Test Reset Option=====
void resetTest(void);
void resetTest(void)
{
  tearDown();
  setUp();
}

char const *progname;


//=======MAIN=====
int main(int argc, char *argv[])
{
  progname = argv[0];
  UnityBegin("packetProcessing.c");
  RUN_TEST(test_TooShortLength, 19);
  RUN_TEST(test_LengthNotMultipleOfFour, 20);
  RUN_TEST(test_TooShortExtensionFieldLength, 21);
  RUN_TEST(test_UnauthenticatedPacketReject, 22);
  RUN_TEST(test_CryptoNAKPacketReject, 23);
  RUN_TEST(test_AuthenticatedPacketInvalid, 24);
  RUN_TEST(test_AuthenticatedPacketUnknownKey, 25);
  RUN_TEST(test_ServerVersionTooOld, 26);
  RUN_TEST(test_ServerVersionTooNew, 27);
  RUN_TEST(test_NonWantedMode, 28);
  RUN_TEST(test_KoDRate, 29);
  RUN_TEST(test_KoDDeny, 30);
  RUN_TEST(test_RejectUnsyncedServer, 31);
  RUN_TEST(test_RejectWrongResponseServerMode, 32);
  RUN_TEST(test_AcceptNoSentPacketBroadcastMode, 33);
  RUN_TEST(test_CorrectUnauthenticatedPacket, 34);
  RUN_TEST(test_CorrectAuthenticatedPacketMD5, 35);
  RUN_TEST(test_CorrectAuthenticatedPacketSHA1, 36);

  return (UnityEnd());
}
