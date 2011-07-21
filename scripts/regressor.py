import numpy

def quat_tilt(quat):
    a0 = quat[0]
    a1 = quat[1]
    a2 = quat[2]
    a3 = quat[3]
    return numpy.array([[0, -a3, a2],
                        [a3, 0, -a1],
                        [-a2, a1, 0]
            ])

def quat_plus(quat):
    if len(quat) != 4:
        raise Exception("len(quat) must be 4, received %s"%str(quat))
    a0 = quat[0]
    a = quat[1:]
    res = numpy.eye(4)
    res[0,0] = a0
    res[1:,0] = a
    res[0,1:] = -1*numpy.array(a)
    res[1:,1:] = a0*numpy.eye(3) + quat_tilt(quat)
    return res


def quat_bar(quat):
    if len(quat) != 4:
        raise Exception("len(quat) must be 4, received %s"%str(quat))
    a0 = quat[0]
    a = quat[1:]
    res = numpy.eye(4)
    res[0,0] = a0
    res[1:,0] = a
    res[0,1:] = -1*numpy.array(a)
    res[1:,1:] = a0*numpy.eye(3) - quat_tilt(quat)
    return res


def main():
    # print quat_plus([1,2,3,4])
    # print quat_bar([1,2,3,4])

if __name__ == '__main__':
    main()
