import numpy as np
import json as js
import sys
from coplanarity import coplanarity, getRotationMatrix
from triangulation import modelTriangulation, baselineTriangulation
import math as mt

def Main():
	modelNo = int(sys.argv[1])
	doCoplanarity(modelNo)

	if modelNo > 1:
		doBaseTriangulation(modelNo)

	mode = 2
	doModelTriangulation(mode, modelNo)


def doModelTriangulation(mode, modelNo):

	cameraVars = getCameraVarsForCoplanarity(modelNo)
	ppl = cameraVars["principalPointLeft"]
	ppr = cameraVars["principalPointRight"]
	f = cameraVars["focalLength"]

	suffix = "Im" + str(modelNo) + "Im" + str(modelNo + 1) + ".json"
	fil = open("outputOfCoplanarity" + suffix, 'r')
	outputCoplanarity = js.loads(fil.read())
	fil.close()

	mr = getRotationMatrix(outputCoplanarity["anglesRight"])
	mr = np.transpose(mr)
	ml = getRotationMatrix(outputCoplanarity["anglesLeft"])
	ml = np.transpose(ml)

	if mode == 1:
		fil = open("pointsForModelSpaceTriangulationForFurtherBLT" + suffix, "r")
	elif mode == 2:
		fil = open("pointsForModelSpaceTriangulation" + suffix, "r")
	pointsNew = js.loads(fil.read())
	fil.close()

	for point in pointsNew:
		pointLeft = point["left"]
		pointLeft = np.array([pointLeft[0] - ppl[0], pointLeft[1] - ppl[1], -1 * f])
		pointLeft = ml @ pointLeft
		point["left"] = pointLeft
		pointRight = point["right"]
		pointRight = np.array([pointRight[0] - ppr[0], pointRight[1] - ppr[1], -1 * f])
		pointRight = mr @ pointRight
		point["right"] = pointRight
	positionOfCameraOne =  getPositionOfCameraOne(modelNo)
	scaledBaseline = getPositionRightCamera(modelNo)
	scaledBaseline = scaledBaseline - positionOfCameraOne
	modelSpace = modelTriangulation(pointsNew, positionOfCameraOne, scaledBaseline)
	print(modelSpace)
	modeldic = {}
	modeldic["modelpoints"] = modelSpace
	modeldic["rightCameraPos"] = positionOfCameraOne + scaledBaseline
	suffix = "Im" + str(modelNo) + "Im" + str(modelNo + 1) + ".json"
	if mode == 1:
		suffix = "ForFurtherBLT" + suffix
	saveModelOutput(suffix, modeldic)

def doCoplanarity(modelNo):
	imageLeftNo = modelNo
	imageRightNo = imageLeftNo + 1
	prefix = "pointsForCoplanarity"
	suffix = "Im" + str(imageLeftNo) + "Im" + str(imageRightNo) + ".json"
	f = open(prefix + suffix, 'r')
	points = js.loads(f.read())
	f.close()
	cameraVars = getCameraVarsForCoplanarity(modelNo) 
	anglesLeft = np.array(cameraVars["anglesLeft"])

	pixLength = cameraVars["pixLength"]
	scalePoints(points, pixLength)

	m = getRotationMatrix(anglesLeft)
	m = np.transpose(m)


	ppl = np.array(cameraVars["principalPointLeft"])
	f = cameraVars["focalLength"]
	for point in points:
		pointLeft = point["left"]
		pointLeft = np.array([pointLeft[0] - ppl[0], pointLeft[1] - ppl[1], -1 *f])
		pointLeft = m @ pointLeft
		point["left"] = pointLeft
		point["right"] = np.array(point["right"])
	cameraVars["baseline"] = np.array([1,0,0])
	cameraVars["anglesRight"] = anglesLeft
	outputCoplanarity = coplanarity(points, cameraVars)
	outputCoplanarity["baseline"] = outputCoplanarity["baseline"]/np.linalg.norm(outputCoplanarity["baseline"]) 
	baseline = outputCoplanarity["baseline"]
	print("Printing whole of the output of coplanarity dictionary")
	print(outputCoplanarity)
	saveCoplanarity(suffix, outputCoplanarity)


def getPositionRightCamera(modelNo):
	if modelNo == 1:
		suffix = "Im" + str(modelNo) +  "Im" + str(modelNo + 1) + ".json"
		fil = open("outputOfCoplanarity" + suffix, 'r')
		oc = js.loads(fil.read())
		fil.close()
		return np.array(oc["baseline"])
	if modelNo > 1:
		suffix = "Im" + str(modelNo) +  "Im" + str(modelNo + 1) + ".json"
		fil = open("outputOfBaselineTriangulation" + suffix, 'r')
		ob = js.loads(fil.read())
		fil.close()
		return np.array(ob["positionRightCamera"])

def scalePoints(points, pixLength):
	for p in points:
		p["left"] = pixLength * np.array(p["left"])
		p["right"] = pixLength * np.array(p["right"])

def doBaseTriangulation(modelNo):

	prefix = "modelspaceForFurtherBLT"
	suffix = "Im" + str(modelNo - 1) + "Im" + str(modelNo) + ".json"
	fil = open(prefix + suffix, "r")
	model = js.loads(fil.read())
	fil.close()
	modelpoints = model["modelpoints"]
	positionOfCameraOne = model["rightCameraPos"]
	model = []
	for mod in modelpoints:
		model.append(np.array(mod))

	fil = open("cameraParameters" + "Im" + str(modelNo + 1) + ".json","r")
	cameraVars = js.loads(fil.read())
	fil.close()
	pp = cameraVars["principalPoint"]
	fl = cameraVars["focalLength"]
	prefix = "outputOfCoplanarity"
	suffix = "Im" + str(modelNo) + "Im" + str(modelNo + 1) + ".json"
	fil = open(prefix + suffix, "r")
	cop = js.loads(fil.read())
	fil.close()
	anglesRight = np.array(cop["anglesRight"])
	baseline = np.array(cop["baseline"])
	rmat = getRotationMatrix(anglesRight)
	rmat = np.transpose(rmat)
	prefix = "pointsForBaselineTriangulation"
	fil = open(prefix + suffix, "r")
	points23 = js.loads(fil.read()) 
	fil.close()
	vectors2d = []
	for points in points23:
		vectors2d.append(np.array(points["right"]))
	vectors3d = []
	for vec in vectors2d:
		vec = vec - pp
		vec = np.array([vec[0], vec[1], -1 * fl])
		vec = rmat @ vec
		vectors3d.append(vec)

	print(model)
	result = baselineTriangulation(positionOfCameraOne, baseline, model, vectors3d)
	print("RESSSULT")
	print(result)
	saveBaselineTriangulation(modelNo, result)

def getPositionOfCameraOne(modelNo):
	if modelNo == 1:
		return np.array([0,0,0])
	elif modelNo == 2:
		suffix = "Im" + str(modelNo - 1) + "Im" + str(modelNo) + ".json"
		fil = open("outputOfCoplanarity" + suffix, 'r')
		oc = js.loads(fil.read())
		fil.close()
		return np.array(oc["baseline"])
	elif modelNo > 2:
		suffix = "Im" + str(modelNo -1) + "Im" + str(modelNo) + ".json"
		fil = open("outputOfBaselineTriangulation" + suffix, 'r')
		ob = js.loads(fil.read())
		fil.close()
		return np.array(ob["positionRightCamera"])
	

def saveBaselineTriangulation(modelNo, result):
	suffix = "Im" + str(modelNo) + "Im" + str(modelNo + 1) + ".json"
	fil = open("outputOfBaselineTriangulation" + suffix, 'w')
	fil.write(js.dumps(result))
	fil.close()

def saveCoplanarity(suffix, outputCoplanarity):
	storeVar = {}
	storeVar["baseline"] = list(outputCoplanarity["baseline"])
	storeVar["anglesRight"] = list(outputCoplanarity["anglesRight"])
	storeVar["anglesLeft"] = list(outputCoplanarity["anglesLeft"])
	fil = open("outputOfCoplanarity" + suffix, "w")
	fil.write(js.dumps(storeVar))
	fil.close()

def saveModelOutput(suffix, modeldic):
	modeldicOut = {}
	modeldicOut["modelpoints"] = []
	for point in modeldic["modelpoints"]:
		point = list(point)
		modeldicOut["modelpoints"].append(point)
	modeldicOut["rightCameraPos"] = list(modeldic["rightCameraPos"])
	fil = open("modelspace" + suffix, "w")	
	fil.write(js.dumps(modeldicOut))
	fil.close()
	
def getCameraVarsForCoplanarity(modelNo):
	cameraVars = {}
	filename = "cameraParametersIm" + str(modelNo) + ".json"	
	f = open(filename, 'r')
	cl = js.loads(f.read())
	f.close()
	cameraVars["principalPointLeft"] = cl["principalPoint"]
	cameraVars["focalLength"] = cl["focalLength"]
	cameraVars["pixLength"] = cl["pixLength"]

	filename = "cameraParametersIm" + str(modelNo + 1) + ".json"	
	f = open(filename, 'r')
	cr = js.loads(f.read())
	f.close()
	print(cr)
	cameraVars["principalPointRight"] = cr["principalPoint"]
	if modelNo > 1:
		suffix = "Im" + str(modelNo-1) + "Im" + str(modelNo) + ".json"
		filename = "outputOfCoplanarity" + suffix
		f = open(filename, 'r')
		oc = js.loads(f.read())
		f.close()
		cameraVars["anglesLeft"] =  oc["anglesRight"]
	else:
		cameraVars["anglesLeft"] = [0,0,0]
	return cameraVars

if __name__ == '__main__':
	Main()
