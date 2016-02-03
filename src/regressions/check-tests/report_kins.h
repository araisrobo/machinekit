#define DREPORT_FWD(ret, joint, carte)  \
        do {            \
            DP("input:      j0(%.2f) j1(%.2f) j2(%.2f) j3(%.2f) j4(%.2f) j5(%.2f)\n", \
                              joint[0], \
                              joint[1], \
                              joint[2], \
                              joint[3], \
                              joint[4], \
                              joint[5]); \
            DP("fwd: ret(%d) x(%.2f) y(%.2f) z(%.2f) a(%.2f) b(%.2f) c(%.2f)\n", \
                    ret, \
                    carte.tran.x, \
                    carte.tran.y, \
                    carte.tran.z, \
                    carte.a, \
                    carte.b, \
                    carte.c); \
        } while (0);

#define DREPORT_INV(ret, joint) \
        do { \
            DP ("inv: ret(%d), j0(%.2f) j1(%.2f) j2(%.2f) j3(%.2f) j4(%.2f) j5(%.2f)\n", \
                  ret, \
                  joint[0], \
                  joint[1], \
                  joint[2], \
                  joint[3], \
                  joint[4], \
                  joint[5]); \
        } while (0);

