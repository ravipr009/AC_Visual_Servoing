import numpy as np
from vsnet_trained import vsnet_predictions
from matplotlib.pyplot import imread
from skimage.transform import resize
from vs_rotation_conversion import rotationMatrixToEulerAngles,eulerAnglesToRotationMatrix


def run_control_law(x,w):
        #Q = Quaternion(q[0],q[1],q[2],q[3]).inverse
        #R = np.transpose(Q.rotation_matrix)
        R = eulerAnglesToRotationMatrix(w)
        #print(eulerAnglesToRotationMatrix(w))
        #print(np.transpose(eulerAnglesToRotationMatrix(-w)))
        t = x
        l1 = 0.1
        l2 = 0.1
        #theta = quaternion_to_euler(q[0],q[1],q[2],q[3])
        #print('1st',theta)

        #theta = rotationMatrixToEulerAngles(Q.rotation_matrix)
        #print('2nd',theta)

        v = -l1*np.matmul(np.transpose(R),t)
        w =  -l2*(w)
        v = np.concatenate((v, w), axis=0)
        return v

def get_next_pose_from_vs(target_img_file,cur_img_file):
    img1 = resize(imread(target_img_file),(224,224))
    img2 = resize(imread(cur_img_file),(224,224))
    pos = vsnet_predictions(img1,img2) #CNN prediction
    print(pos)
    x = pos[0:3]
    w = pos[3:6]
    v = run_control_law(-x,-w)
    print('next_pose:',v)
    return v
