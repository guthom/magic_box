#!/usr/bin/env python
import sys, os

#find better way to do this.
projectDir = os.path.join(os.path.dirname(__file__), '../')
sys.path.append(projectDir + "display/")

import rospy
import PIL.Image as Image
import nodeconfig as config
import json
from magic_box.srv import *

from display.EInkDisplay import EInkDisplay

class MagicBox:
    _nodeTopic = None
    _boxName = None
    _currentID = None
    _services = dict()
    _display = None

    def LoadSaves(self):
        with open(projectDir + "../" + config.saveBoxFilePath) as data_file:
            data = json.load(data_file)

        self._currentID = data["activeID"]
        rospy.loginfo("Last known ID: " + str(self._currentID))

        #self.ShowAprilTag(data["activeID"])

    def DumpSave(self):
        data = dict()
        data["activeID"] = self._currentID
        with open(projectDir + "../" + config.saveBoxFilePath, "w") as outfile:
            json.dump(data, outfile)

    def Init(self):
        rospy.init_node(config.nodeName, disable_signals=True)
        self._nodeTopic = rospy.get_name() + "/"
        self.InitParams()
        self._display = EInkDisplay()

        self.LoadSaves()

    def InitParams(self):
        self._boxName = rospy.get_param(self._nodeTopic + "/boxName", default="magic_box")
        self._masterMode = rospy.get_param(self._nodeTopic + "/masterMode", default=True)

    def ShowAprilTag(self, id):
        try:
            filename = config.aprilTagRawName + str(id).zfill(5) + ".png"
            path = projectDir + "../" + config.aprilTagPath + filename
            rospy.loginfo("Showing TagID: " + str(id) + " with filename: " + filename)
            image = Image.open(path)

            if config.upsideDown:
                image = Image.rotate(180)

            self._display.ShowImage(image)

            self._currentID = id

            self.DumpSave()

            return True
        except ValueError:
            print("Could not convert data to the needed ID")
        except:
            print("Exception during ID-Change")
            return False

    def DemoLoop(self):
        rospy.loginfo("Started MagicBox in DemoMode!")
        id = 0
        counter = 0
        while not rospy.is_shutdown():

            if counter % 1000 == 0:
                self.ShowAprilTag(id)
                id += 1

            counter += 1
            self.rate.sleep()

    def RegisterBox(self):
        serviceTopic = "/" + config.boxMasterTopic + "/registerBox"

        rospy.loginfo("Waiting for MagicBoxMaster Sercice : " + serviceTopic + " to register " + self._boxName)
        rospy.wait_for_service(serviceTopic)

        service = rospy.ServiceProxy(serviceTopic, RegisterMagicBox, persistent=True)
        request = RegisterMagicBoxRequest()
        request.boxName = self._boxName
        request.baseTopic = self._nodeTopic

        response = service.call(request)

        if response.success is True:
            rospy.loginfo("Successful registered Box: " + self._boxName)
        else:
            rospy.logerr("Can't register Box: " + self._boxName + " please try again!")



    def InitServices(self):
        self._services["setNewID"] = rospy.Service(self._nodeTopic + "setNewID", SetNewBoxId,
                                                   self.HandleIDChangeService)
        self._services["getActiveID"] = rospy.Service(self._nodeTopic + "getActiveID", GetActiveID,
                                                       self.HandleGetIDService)

        for service in self._services:
            rospy.wait_for_service(self._nodeTopic + service)
            rospy.loginfo("Service " + str(service) + " is ready now!")

    def HandleIDChangeService(self, req):
        ret = SetNewBoxIdResponse()

        if self.ShowAprilTag(req.id):
            ret.success = True
        else:
            ret.success = False

        return ret

    def HandleGetIDService(self, req):
        ret = GetActiveIDResponse()
        print str(self._currentID)
        ret.activeID = int(self._currentID)

        return ret

    def ServiceLoop(self):
        self.InitServices()
        rospy.loginfo("Started MagicBox in ServiceMode!")
        rospy.loginfo("The name of this magic box is: " + self._boxName)

        if self._masterMode:
            self.RegisterBox()

        while not rospy.is_shutdown():
            self.rate.sleep()

    def node(self):

        self.Init()

        self.rate = rospy.Rate(config.rate)

        print("Start Node: " + self._nodeTopic)

        if config.demoMode:
            self.DemoLoop()
        else:
            self.ServiceLoop()

