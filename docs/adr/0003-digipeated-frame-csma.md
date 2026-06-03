# Digipeated frames use p=0 CSMA; beacons use standard p-persist

Digipeated frames (relayed from RF) and beacons (locally-originated) use different CSMA policies.

A future reader might expect all transmissions to share the same channel access behavior. The reason they don't: digipeaters serve a relay function. A digipeater that waits for a random backoff before relaying adds latency that compounds with every hop. On a busy channel, a frame might take seconds to traverse a 3-hop path. The APRS network depends on digipeaters being responsive.

Beacons, however, are locally-generated periodic messages (station ID, status). They can afford standard p-persist CSMA with random backoff because they aren't time-critical.

The digipeated frame CSMA policy is: wait for DCD to clear, then transmit immediately (p=0, slot=10ms). Beacons use the standard p-persist and slot time configured for the TNC.

## Considered Options

**Same CSMA for all transmissions.** Rejected — adds per-hop latency that degrades APRS network performance. A 3-hop path with 200ms random backoff per hop adds up to 600ms of unnecessary delay.

**Split CSMA: digipeated p=0, beacons standard.** Selected — digipeated frames relay as fast as the channel allows; beacons coexist fairly with other stations.
