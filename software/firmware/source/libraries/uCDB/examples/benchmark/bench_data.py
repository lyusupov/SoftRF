#!/usr/bin/env python2

'''
    Based on - https://www.unixuser.org/~euske/doc/cdbinternals/pycdb.py.html
'''

# calc hash value with a given key
def calc_hash(s):
    return reduce(lambda h, c: (((h << 5) + h) ^ ord(c)) & 0xffffffffL, s, 5381)

# cdbmake(filename, hash)
def cdbmake(f, a):
    from struct import pack

    # write cdb
    def write_cdb(fp):
        pos_header = fp.tell()

        # skip header
        p = pos_header + 8 * 256
        fp.seek(p)

        bucket = [[] for i in range(256)]

        # write data & make hash
        for (k, v) in a.iteritems():
          fp.write(pack('<LL', len(k), len(v)))
          fp.write(k)
          fp.write(v)
          h = calc_hash(k)
          bucket[h % 256].append((h, p))
          p += len(k) + len(v) + 8

        pos_hash = p

        # write hash tables
        for bt in bucket:
            if bt:
                nslots = 2 * len(bt)
                hash_tab = [(0, 0) for i in range(nslots)]
                for (h, p) in bt:
                    i = (h >> 8) % nslots
                    while hash_tab[i][1]:  # is slot already occupied?
                        i = (i + 1) % nslots
                    hash_tab[i] = (h, p)
                for (h, p) in hash_tab:
                    fp.write(pack('<LL', h, p))

        # write header
        fp.seek(pos_header)
        for bt in bucket:
            fp.write(pack('<LL', pos_hash, 2 * len(bt)))
            pos_hash += 16 * len(bt)
        return

    fp = file(f, "wb")
    write_cdb(fp)
    fp.close()
    return

if __name__ == "__main__":

    bench = {}
    for i in xrange(0, 5*1000000, 5):
        bench[str(i)] = str(i)
    print bench['255']
    cdbmake("bench.cdb", bench)
