#!/usr/bin/env python
import sys, os

projectDir = os.path.join(os.path.dirname(__file__), '../')

import rospy
import nodeconfig as config
import json
from magic_box.srv import *

class MagicBoxInformation:

    name = ""
    baseServiceTopic = ""

    def __init__(self, name, baseServiceTopic):
        self.name = name
        self.baseServiceTopic = baseServiceTopic

    def ChangeID(self, newId):
        service = rospy.ServiceProxy(self.baseServiceTopic + "setNewID", SetNewBoxId, persistent=True)
        request = SetNewBoxIdRequest()
        request.id = newId
        response = service.call(request)

        return response.success

    def GetID(self):
        service = rospy.ServiceProxy(self.baseServiceTopic + "getActiveID", GetActiveID, persistent=True)
        request = GetActiveIDRequest()
        response = service.call(request)

        return response.activeID

    def ToJson(self):
        data = dict()
        data["name"] = self.name
        data["baseServiceTopic"] = self.baseServiceTopic

        return data



class MagicBoxMaster:

    _registeredBoxes = None
    _services = None

    def Init(self):
        # init node disable signal for internal shutdown
        rospy.init_node(config.boxMasterTopic, disable_signals=True)
        self._nodeTopic = rospy.get_name() + "/"

        self._services = dict()
        self._registeredBoxes = dict()

        self.LoadSaves()

        self.InitParams()
        self.InitServices()

    def LoadSaves(self):
        path = projectDir + "../" + config.saveMasterFilePath
        if os.path.isfile(path):
            with open(path) as data_file:
                data = json.load(data_file)

            list = []
            for box in data:
                box = data[box]
                list.append(str(box["name"]))
                self._registeredBoxes[str(box["name"])] = MagicBoxInformation(name=str(box["name"]),
                                                                         baseServiceTopic=str(box["baseServiceTopic"]))

            rospy.loginfo("Restored following " + str(list.__len__()) + " Boxes: " + str(list))
        else:
            rospy.logwarn("Can't find save file for known boxes!")



    def DumpSave(self):

        data = dict()

        for box in self._registeredBoxes:
            data[box] = self._registeredBoxes[box].ToJson()

        with open(projectDir + "../" + config.saveMasterFilePath, "w") as outfile:
            json.dump(data, outfile)

    def InitParams(self):
        pass

    def InitServices(self):
        self._services["changeID"] = rospy.Service(self._nodeTopic + "changeID", ChangeBoxID,
                                                   self.HandleChangeIDService)

        self._services["registerBox"] = rospy.Service(self._nodeTopic + "registerBox", RegisterMagicBox,
                                                      self.HandleRegistrationService)

        self._services["deRegisterBox"] = rospy.Service(self._nodeTopic + "deRegisterBox", DeRegisterMagicBox,
                                                        self.HandleDeRegistrationService)

        for service in self._services:
            rospy.wait_for_service(self._nodeTopic + service)
            rospy.loginfo("Service " + str(service) + " is ready now!")

    def HandleChangeIDService(self, req):
        ret = ChangeBoxIDResponse()

        if not self._registeredBoxes.__contains__(req.boxName):
            rospy.logerr("Selected Box is not registered!")
            ret.success = False
        else:
            if self._registeredBoxes[req.boxName].ChangeID(req.tagID):
                rospy.loginfo("Changed ID of " + req.boxName + ". to ID: " + str(req.tagID))
                ret.success = True
            else:
                ret.success = False

        return ret

    def HandleRegistrationService(self, req):
        ret = RegisterMagicBoxResponse()

        if self._registeredBoxes.__contains__(req.boxName):
            rospy.logwarn("Box is allready registered. Known entry will be overwritten!")

        self._registeredBoxes[req.boxName] = MagicBoxInformation(name=req.boxName, baseServiceTopic=req.baseTopic)
        rospy.loginfo("Registered Box with name: " + req.boxName)
        ret.success = True

        self.DumpSave()
        return ret

    def HandleDeRegistrationService(self, req):
        ret = DeRegisterMagicBoxResponse()

        if not self._registeredBoxes.__contains__(req.boxName):
            rospy.logerr("Can't deregister box! Box is not yet registered")
            ret.success = False
        else:
            self._registeredBoxes.pop(req.boxName)
            rospy.logerr("Derigistered Box: " + req.boxName)
            ret.success = True

        self.DumpSave()
        return ret

    def node(self):

        self.Init()

        self.rate = rospy.Rate(config.rate)

        rospy.loginfo("Started MagicBoxMaster")
        while not rospy.is_shutdown():
            self.rate.sleep()

