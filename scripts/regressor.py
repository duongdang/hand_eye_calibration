import numpy
import pickle
import roslib, os, sys
roslib.load_manifest('hand_eye_calibration')
import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped, Transform, PoseStamped
import itertools
import random
from tf import transformations
import tf
from numpy import linalg
import math

def eig(A):
    """
    Return eigenvalues and eigenvectors in ascending order of realpart then
    imag part of eigenvalues.
    """
    lambdas, vs = linalg.eig(A)
    lvs = [(lambdas[i], vs[i]) for i in range(len(vs))]

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
    if 'position' in dir(data):
        translation = [data.position.x, data.position.y, data.position.z]
        rotation = [data.orientation.x, data.orientation.y,
                       data.orientation.z, data.orientation.w]
    else:
        translation = [data.translation.x, data.translation.y,
                       data.translation.z]
        rotation = [data.rotation.x, data.rotation.y,
                       data.rotation.z, data.rotation.w]
    res = transformations.quaternion_matrix(rotation)
    res[:3,3] = translation
    return res



def regress(data):
    # make a copy
    data = data[:]
    # randomize the list
    random.shuffle(data)
    N = len(data)

    # Optimization of quaternion
    E = None
    A2 = numpy.zeros((4,4))
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

        Rl = Hl[:3,:3]
        Rc = Hc[:3,:3]
        lambdas_l, V_l  = eig(Rl)
        lambdas_c, V_c   = eig(Rc)

        for i in range(2,3):
            v_prime = numpy.zeros(4)
            v = numpy.zeros(4)
            print lambdas_c[i], lambdas_l[i]
            for j in range(3):
                v[1+j]       = V_c[i][j]
                v_prime[1+j] = V_l[i][j]

            Ei = (quat_plus(v_prime) - quat_bar(v))

            A2i = numpy.dot(numpy.transpose(Ei), Ei)

            A2 += A2i


    Et = numpy.transpose(E)
    A = numpy.dot(Et, E)
    lambdas, vs = eig(A)
    print "Chou"
    print "eigenvalues: ", lambdas

    qx = vs[0]
    #print "E*qx=", numpy.dot(E,qx)
    # qx = [1., 0., 1., 0. ]
    #if qx[0] < 0:
    #    qx = -qx
    Ax = transformations.quaternion_matrix(qx)
    Ax = Ax[:3,:3]
    print "qx: ", qx
    print "rpy(rad): ", [1*a for a in transformations.euler_from_matrix(Ax)]
    print "rpy(deg): ", [180*a/math.pi for a in transformations.euler_from_matrix(Ax)]
    print "---"

    lambdas, vs = eig(A2)
    print "Horaud"
    print "eigenvalues: ", lambdas

    qx = vs[0]
    #print "E*qx=", numpy.dot(E,qx)
    # qx = [1., 0., 1., 0. ]
    if qx[0] < 0:
        qx = -qx
    Ax = transformations.quaternion_matrix(qx)
    Ax = Ax[:3,:3]
    print "qx: ", qx
    print "rpy(rad): ", [1*a for a in transformations.euler_from_matrix(Ax)]
    print "rpy(deg): ", [180*a/math.pi for a in transformations.euler_from_matrix(Ax)]

    # Optimization of translation
    A = None
    b = []
    for i in range(len(Hls)):
        Hl = Hls[i]
        Hc = Hcs[i]
        rl = Hl[:3,3]
        rc = Hc[:3,3]
        Al = Hl[:3,:3]
        # print Al - numpy.eye(3)
        if A == None:
            A = Al - numpy.eye(3)
        else:
            A = numpy.append(A, Al - numpy.eye(3), axis = 0)

        b = numpy.append(b, numpy.dot(Ax, rc) - rl)

    # leastquare A*rx = b
    # print A
    # print b
    #rx, res, rank, s = linalg.lstsq(A, b, 1e-6)
    #print "residual: ", res
    #print "rx: ", rx
    #print numpy.dot(A, rx) -b



def main():
    # print quat_plus([1,2,3,4])
    # print quat_bar([1,2,3,4])
    data = pickle.load(open('pos.pickle'))
    regress(data)

if __name__ == '__main__':
    main()
