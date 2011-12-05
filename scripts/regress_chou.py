import numpy
import pickle
import roslib, os, sys
roslib.load_manifest('hand_eye_calibration')
import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped, Transform, PoseStamped
import itertools
import random
from robotviewer import transformations
from numpy import linalg
import math
import doctest

def eig(A):
    """
    Return eigenvalues and eigenvectors in ascending order of realpart then
    imag part of eigenvalues.
    """
    lambdas, vs = linalg.eig(A)
    lvs = [(lambdas[i], vs[:,i]) for i in range(len(lambdas))]

    def lv_cmp(lv1, lv2):
        lambda1 = lv1[0]
        lambda2 = lv2[0]

        r1 = numpy.real(lambda1)
        i1 = numpy.real(lambda1)
        r2 = numpy.real(lambda2)
        i2 = numpy.real(lambda2)

        if r1 > r2:
            return 1
        elif r1 < r2:
            return -1
        elif i1 > i2:
            return 1
        elif i1 < i2:
            return -1
        return 0

    lvs.sort(cmp = lv_cmp)

    lambdas = [lv[0] for lv in lvs]
    vs = [lv[1] for lv in lvs ]
    return lambdas, vs


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
    av = quat[1:]
    res = numpy.eye(4)
    res[0,0] = a0
    res[1:,0] = av
    res[0,1:] = -1*numpy.array(av)
    res[1:,1:] = a0*numpy.eye(3) + quat_tilt(quat)
    # print quat
    # print "----------------------------------------------------"
    # print res
    # print "====================================================\n\n"
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

def pos_to_mat(data):
    if 'pose' in dir(data):
        data = data.pose
    if 'position' in dir(data):
        translation = [data.position.x, data.position.y, data.position.z]
        rotation = [data.orientation.w, data.orientation.x, data.orientation.y,
                       data.orientation.z, ]
    else:
        translation = [data.translation.x, data.translation.y,
                       data.translation.z]
        rotation = [ data.rotation.w, data.rotation.x, data.rotation.y,
                       data.rotation.z,]
    res = transformations.quaternion_matrix(rotation)
    res[:3,3] = translation
    return res



def regress_pose(data):
    # make a copy
    data = data[:]
    # randomize the list
    random.shuffle(data)
    N = len(data)

    Hls = []
    Hcs = []


    for i in range(N-1):
        # each datum: chessboard_pos, link_pos
        datum_bef = data[i]
        Tg_bef = pos_to_mat(datum_bef[0])
        Tl_bef = pos_to_mat(datum_bef[1])

        datum_aft = data[i+1]
        Tg_aft = pos_to_mat(datum_aft[0])
        Tl_aft = pos_to_mat(datum_aft[1])

        Hl = numpy.dot(numpy.linalg.inv(Tl_bef), Tl_aft)
        Hc = numpy.dot(Tg_bef, numpy.linalg.inv(Tg_aft))
        # print Hl
        # print Hc
        # print "===============\n\n"

        Hls.append(Hl)
        Hcs.append(Hc)

    return regress_Hs(Hls, Hcs)


def regress_pose2(link_poses, chessboard_poses):
    N = len(link_poses)
    Hls = []
    Hcs = []
    for i in range(N):
        for j in range(i+1, N):
            Tl_bef = link_poses[i]
            Tl_aft = link_poses[j]
            Tg_bef = chessboard_poses[i]
            Tg_aft = chessboard_poses[j]
            Hl = numpy.dot(numpy.linalg.inv(Tl_bef), Tl_aft)
            Hc = numpy.dot(Tg_bef, numpy.linalg.inv(Tg_aft))
            Hls.append(Hl)
            Hcs.append(Hc)

    return regress_Hs(Hls, Hcs)

def rotation_from_matrix(M):
    th, u, p = transformations.rotation_from_matrix(M)
    if th < 0:
        th = -th
        u = -u
    return th, u, p

def regress_Hs(Hls, Hcs):
    """
    >>> Hx  = numpy.array([ \
    [-0.88405797,-0.40579710, -0.23188406, 11.00000000], \
    [-0.40579710, 0.42028986,  0.81159420, 21.00000000], \
    [-0.23188406, 0.81159420, -0.53623188,-18.00000000], \
    [ 0         , 0         , 0          , 1] \
    ])
    >>> Hl1 = numpy.array([ \
    [-0.87179487, 0.48717949, -0.05128205,  5.00000000], \
    [ 0.33333333, 0.66666667,  0.66666667, -4.00000000], \
    [ 0.35897436, 0.56410256, -0.74358974,  3.00000000], \
    [ 0         , 0         , 0          , 1] \
    ])
    >>> Hl2 = numpy.array([ \
    [-0.70114943, 0.02298850, -0.71264368,  2.00000000], \
    [ 0.66666667,-0.33333333, -0.66666667, -3.00000000], \
    [-0.25287356,-0.94252874,  0.21839080,  9.00000000], \
    [ 0         , 0         , 0          , 1] \
    ])
    >>> Hc1 = numpy.array([ \
    [-0.13831397,-0.61660716, -0.77502572,  0.131178  ], \
    [-0.84328869,-0.33704404,  0.41864724, 34.399851  ], \
    [-0.51935868, 0.71147518, -0.47335997,-41.570048  ], \
    [ 0         , 0         , 0          , 1] \
    ])
    >>> Hc2 = numpy.array([ \
    [-0.69307617, 0.66439727, -0.27968142,  7.6275196 ], \
    [ 0.01005777,-0.37903028, -0.92532961, -3.1216059 ], \
    [-0.72079419,-0.64413687,  0.25601450, -8.9446943 ], \
    [ 0         , 0         , 0          , 1] \
    ])
    >>> numpy.linalg.norm(numpy.dot(Hl1, Hx) - numpy.dot(Hx, Hc1)) < 1e-6
    True
    >>> numpy.linalg.norm(numpy.dot(Hl2, Hx) - numpy.dot(Hx, Hc2)) < 1e-6
    True
    >>> exp_ul1 = -numpy.array([ 0.22798115, 0.91168461, 0.34188173])
    >>> exp_ul2 = numpy.array([-0.32929278,-0.54882130, 0.76834982])
    >>> exp_uc1 = -numpy.array([-0.65073141, 0.56815128, 0.50373878])
    >>> exp_uc2 = numpy.array([ 0.33565393, 0.52655029,-0.78107611])
    >>> exp_th1 = 360-193.002824
    >>> exp_th2 = 155.239701
    >>> thl1, ul1, pl1 = rotation_from_matrix(Hl1)
    >>> thl2, ul2, pl2 = rotation_from_matrix(Hl2)
    >>> thc1, uc1, pc1 = rotation_from_matrix(Hc1)
    >>> thc2, uc2, pc2 = rotation_from_matrix(Hc2)
    >>> thl1 *= 180./math.pi
    >>> thl2 *= 180./math.pi
    >>> thc1 *= 180./math.pi
    >>> thc2 *= 180./math.pi
    >>> if not numpy.linalg.norm(ul1 - exp_ul1) < 1e-4: print ul1
    >>> if not numpy.linalg.norm(ul2 - exp_ul2) < 1e-4: print ul2
    >>> if not numpy.linalg.norm(uc1 - exp_uc1) < 1e-4: print uc1
    >>> if not numpy.linalg.norm(uc2 - exp_uc2) < 1e-4: print uc2
    >>> if not abs(thl1 - exp_th1) < 1e-2: print thl1
    >>> if not abs(thl2 - exp_th2) < 1e-2: print thl2
    >>> if not abs(thc1 - exp_th1) < 1e-2: print thc1
    >>> if not abs(thc2 - exp_th2) < 1e-2: print thc2

    >>> Hx_reg = regress_Hs([Hl1, Hl2], [Hc1, Hc2])
    >>> numpy.allclose(Hx_reg[:3,:3], Hx[:3,:3])
    True
    >>> numpy.allclose(Hx_reg[:3,3], Hx[:3,3])
    True
    """

    E = None
    A2 = numpy.zeros((4,4))

    for i in range(len(Hls)):
        Hl = Hls[i]
        Hc = Hcs[i]
        pl = transformations.quaternion_from_matrix(Hl)
        pc = transformations.quaternion_from_matrix(Hc)

        lhA = quat_plus(pl) - quat_bar(pc)

        if E == None:
            E = lhA
        else:
            E = numpy.append(E, lhA, axis = 0)

    Et = numpy.transpose(E)
    A = numpy.dot(Et, E)
    lambdas, vs = eig(A)
    qx = vs[0]
    Ax = transformations.quaternion_matrix(qx)

    # Optimization of translation
    A = None
    b = []
    for i in range(len(Hls)):
        Hl = Hls[i]
        Hc = Hcs[i]
        rl = Hl[:3,3]
        rc = Hc[:3,3]
        Al = Hl[:3,:3]
        if A == None:
            A = Al - numpy.eye(3)
        else:
            A = numpy.append(A, Al - numpy.eye(3), axis = 0)

        b = numpy.append(b, numpy.dot(Ax[:3,:3], rc) - rl, axis = 0)
        # print rc
        # print Ax
        # print numpy.dot(Ax[:3,3], rc)
        # print "rl:",rl
        # print Al
        # print "---"

    # leastquare A*rx = b
    # print A
    # print b

    rx, res, rank, s = linalg.lstsq(A, b, 1e-6)
    #print rx, res, rank, s
    Ax[:3,3] = rx
    return Ax, E, lambdas, res

def test():
    import doctest
    doctest.testmod()

def main():
    # print quat_plus([1,2,3,4])
    # print quat_bar([1,2,3,4])
    test()
    data = pickle.load(open('pos.pickle'))
    print regress_pose(data)


if __name__ == '__main__':
    main()
