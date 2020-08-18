import numpy as np
import math as mt
import json as js

def getRotationMatrix(angle):
	omega = angle[0]
	phi = angle[1]
	kappa = angle[2]
	m = np.zeros((3,3))
	m[0,0] = mt.cos(phi) * mt.cos(kappa)
	m[0,1] = mt.sin(omega) * mt.sin(phi) * mt.cos(kappa) + mt.cos(omega)*mt.sin(kappa)
	m[0,2] = -1 * mt.cos(omega) * mt.sin(phi) * mt.cos(kappa) + mt.sin(omega)*mt.sin(kappa)
	m[1,0] = -1 * mt.cos(phi) * mt.sin(kappa)
	m[1,1] = -1 * mt.sin(omega) * mt.sin(phi) * mt.sin(kappa) + mt.cos(omega)*mt.cos(kappa)
	m[1,2] = mt.cos(omega) * mt.sin(phi) * mt.sin(kappa) + mt.sin(omega)*mt.cos(kappa)
	m[2,0] = mt.sin(phi)
	m[2,1] = -1 * mt.sin(omega)*mt.cos(phi)
	m[2,2] = mt.cos(omega)*mt.cos(phi)
	return m

def getMatrixBee(cameraVars, pointVars):
	noOfPoints = len(pointVars)
	B = np.zeros((noOfPoints, 6))
	for i in range(noOfPoints):
		B[i] = np.array(blist(pointVars[i], cameraVars))
	return B

def blist(pointVar, cameraVars):
	a1  = pointVar["left"]
	pointRight = pointVar["right"]
	f = cameraVars["focalLength"]
	ppr = cameraVars["principalPointRight"]

	r2 = np.array([pointRight[0] - ppr[0], pointRight[1] - ppr[1], -1*f])
	m = cameraVars["rotationMatrix2"]
	m= np.transpose(m)
	b = cameraVars["baseline"]
	a2 = m @ r2

	omega = cameraVars["anglesRight"][0]
	phi = cameraVars["anglesRight"][1]
	kappa = cameraVars["anglesRight"][2]
	
	b1 = -1 * np.linalg.det(np.array([[a1[0], a1[2]], [a2[0], a2[2]]]))

	b2 = np.linalg.det(np.array([[a1[0], a1[1]], [a2[0], a2[1]]]))

	b3 = np.linalg.det(np.array([[b[0], b[1], b[2]], [a1[0], a1[1], a1[2]], [0, -1*a2[2], a2[1]]]))

	diff1 = -1*r2[0]*mt.sin(phi)*mt.cos(kappa) + r2[1]*mt.sin(phi)*mt.sin(kappa) + r2[2]*mt.cos(phi)

	diff2 = r2[0]*mt.sin(omega)*mt.cos(phi)*mt.cos(kappa) - r2[1]*mt.sin(omega)*mt.cos(phi)*mt.sin(kappa)
	diff2 = diff2 + r2[2]*mt.sin(omega)*mt.sin(phi)

	diff3 = -1*r2[0]*mt.cos(omega)*mt.cos(phi)*mt.cos(kappa) + r2[1]*mt.cos(omega)*mt.cos(phi)*mt.sin(kappa) 
	diff3 = diff3 - r2[2]*mt.cos(omega)*mt.sin(phi)

	b4 = np.linalg.det(np.array([[b[0], b[1], b[2]], [a1[0], a1[1], a1[2]], [diff1, diff2, diff3]]))

	diff1 = r2[0]*m[1,0] - r2[1]*m[0,0]
	diff2 = r2[0]*m[1,1] - r2[1]*m[0,1]
	diff3 = r2[0]*m[1,2] - r2[1]*m[0,2]
	b5 = np.linalg.det(np.array([[b[0], b[1], b[2]], [a1[0], a1[1], a1[2]], [diff1, diff2, diff3]]))

	F = -1 * np.linalg.det(np.array([[b[0], b[1], b[2]], [a1[0], a1[1], a1[2]], [a2[0], a2[1], a2[2]]]))

	l = [b1, b2, b3, b4, b5, F]
	return l

def coplanarity(pointVars, cameraVars):
	cameraVars["rotationMatrix2"] = getRotationMatrix(cameraVars["anglesRight"])
	threshold = 0.001
	count = 0
	while(True):
		B = getMatrixBee(cameraVars, pointVars) 
		F = B[:,5]
		#if (count > 0) and (np.linalg.norm(F) > oldNorm):
		#	break
		#oldNorm = np.linalg.norm(F)
		B = B[:,0:5]
		delta = np.linalg.lstsq(B, F, rcond=None)[0]
		if np.linalg.norm(delta) < threshold:
			break
		#print("Determinants are")
		#print(F)
		makeChanges(cameraVars, delta)
		count = count + 1
		#print("After " + str(count) + "th iteration")
		#print("The delta vector is:")
		#print(delta)
		#print("The norm of determs is:")
		#print(np.linalg.norm(F))
		#print("\n")

	#print("angles of the right photo in degrees are")
	#print(cameraVars["anglesRight"]*180/mt.pi)
	#print("baseline is")
	#print(cameraVars["baseline"])
	#print("\n")
	#print("Results of coplananrity ends")
	return cameraVars


def makeChanges(cameraVars, delta):
	deltaAngles = delta[2:]
	cameraVars["baseline"] = cameraVars["baseline"] + np.array([0, delta[0], delta[1]])
	cameraVars["anglesRight"] = cameraVars["anglesRight"] + deltaAngles
	cameraVars["rotationMatrix2"] = getRotationMatrix(cameraVars["anglesRight"])

if __name__ == '__main__':
	Main()
