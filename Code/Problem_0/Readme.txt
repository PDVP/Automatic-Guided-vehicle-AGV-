Problem 0 :::
Need to bserve the encder data whfen mtor spins..

In psrt.ino the prgram is not reliable as the data keeps togling from 0-32600- -32600-0 .
so the problem is with volatile int as the encider value is very large , so debugged by using volatile long.
Along with it the atomic block library is also not giving result as expected , so removed it.

Hence the final code is written in WATE.ino  .