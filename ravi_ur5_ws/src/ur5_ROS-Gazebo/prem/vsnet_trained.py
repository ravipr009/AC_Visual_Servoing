import torch 
import torch.nn as nn
from resnet1_0 import resnet18
from matplotlib.pyplot import imread
from scipy.misc import imsave 
from skimage.transform import resize
import torchvision.transforms as transforms
import numpy as np


#from servo_dataset import Servo_Dataset
#from torch.utils import data
#import time
#import matplotlib.pyplot as plt
import torch.nn.functional as F
import sys

# Ignore warnings
import warnings
warnings.filterwarnings("ignore")


device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

class ArrayToTensor(object):
	"""Converts a numpy.ndarray (H x W x C) to a torch.FloatTensor of shape (C x H x W)."""

	def __call__(self, array):
		assert(isinstance(array, np.ndarray))
		array = np.transpose(array, (2, 0, 1))
		# handle numpy array
		tensor = torch.from_numpy(array)
		# put it from HWC to CHW format
		return tensor.float()

class rank_pool(nn.Module):
	def __init__(self,in_planes, out_planes, size):
		super(rank_pool, self).__init__()
		self.exp = 4 # Expension Ratio (E)
		self.p1 = nn.Conv2d(in_planes, out_planes*self.exp, 3, 2, 1, groups=in_planes, bias=False) # DepthWise Convolution using 3x3 kernel
		self.bn1 = nn.BatchNorm2d(out_planes*self.exp)

		self.p2 = nn.Conv2d(out_planes*self.exp, out_planes, kernel_size=1, stride=1, padding=0, bias=False) # PointWise Convolution using 1x1 kernel
		self.bn2 = nn.BatchNorm2d(out_planes)

		self.p3 = nn.Conv2d(out_planes, out_planes, size, 1, 0, groups=out_planes, bias=False) # Ranking using DepthWise Convolution (WxH kernel) 
		self.bn3 = nn.BatchNorm2d(out_planes) 
		self.sig = nn.Sigmoid()
	
	def forward(self, x):
		out = F.relu(self.bn1(self.p1(x)))
		out = self.bn2(self.p2(out))

		y = self.sig(self.bn3(self.p3(out)))
		out = out*y.view(x.size(0),-1,1,1) 
		out = F.relu(out)       
		return out

class VSNet(nn.Module):
	

	def __init__(self,batchNorm=True):
		super(VSNet,self).__init__()
		self.batchNorm = batchNorm
		self.base_cnn = resnet18(pretrained=True)  #Remove the extra layers
		#self.base_cnn2 = resnet18(pretrained=True)
		self.conv_block = nn.Sequential(
			nn.Conv2d(128,256, kernel_size=3, stride=1,padding=1, bias=False),
			nn.BatchNorm2d(256),
			nn.ReLU(inplace=True),
			#nn.MaxPool2d(kernel_size=3, stride=2, padding=1),
			rank_pool(256,256,28),
			nn.Conv2d(256,512, kernel_size=3, stride=1,padding=1, bias=False),
			nn.BatchNorm2d(512),
			nn.ReLU(inplace=True),
			#nn.MaxPool2d(kernel_size=3, stride=2, padding=1),
			rank_pool(512,512,14),
			nn.Conv2d(512,1024, kernel_size=3, stride=1,padding=1, bias=False),
			nn.BatchNorm2d(1024),
			nn.ReLU(inplace=True),
			#nn.MaxPool2d(kernel_size=3, stride=2, padding=1),
			rank_pool(1024,1024,7),
			nn.AdaptiveAvgPool2d((1, 1))
			)

		

		#self.bn1 = nn.BatchNorm1d(200)
		#self.relu = nn.ReLU(inplace=True)
		self.fc_feature = nn.Linear(1024,4096)
		self.fc = nn.Linear(4096,6)
		#torch.nn.init.xavier_uniform(self.fc.weight)

	def forward(self, x1,x2):
		v1 = self.base_cnn(x1)
		v2 = self.base_cnn(x2)
		#print('v1',v1.shape)
		v = torch.cat((v1,v2),dim=1)
		#print('v',v.shape)
		v = self.conv_block(v)
		v = v.view(v.size(0), -1)
		#print(v.shape)
		v = self.fc_feature(v)
		out = self.fc(v)
		return out


def vsnet_predictions(img1,img2):

	input_transform = transforms.Compose([
	ArrayToTensor(),
	#transforms.Normalize(mean=[0,0,0], std=[255,255,255]),
	transforms.Normalize(mean=[0.4720, 0.4711, 0.4666], std=[0.1172, 0.1175, 0.1159])
	])

	model = VSNet().to(device)
	model.load_state_dict(torch.load('vsnet_trained_weights.pt',map_location='cpu'))
	model.eval()
	img1 = input_transform(img1).unsqueeze(0).to(device)
	img2 = input_transform(img2).unsqueeze(0).to(device)
	pos = model(img1,img2).squeeze(0).detach().numpy()
	return pos


if __name__ == '__main__':

	img1 = sys.argv[1]
	img2 = sys.argv[2]
	img1 = resize(imread(img1),(224,224))
	img2 = resize(imread(img2),(224,224))
	#print(img1)
	pos = vsnet_predictions(img1,img2)
	print(pos)



	
	
