# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# Copyright (c) 2014, Aldebaran Robotics (c)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import actionlib
import rospy
from nao_interaction_msgs.srv import (
    SetAudioMasterVolume,
    SetAudioMasterVolumeRequest
    )

from distutils.version import LooseVersion
import python_qt_binding
if LooseVersion(python_qt_binding.QT_BINDING_VERSION).version[0] >= 5:
    from python_qt_binding.QtWidgets import QComboBox, QMessageBox
else:
    from python_qt_binding.QtGui import QComboBox, QMessageBox
from naoqi_bridge_msgs.msg import BodyPoseWithSpeedAction, BodyPoseWithSpeedGoal

class VolumeWidget(QComboBox):
    def __init__(self):
        super(VolumeWidget, self).__init__()
        self.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.setInsertPolicy(QComboBox.InsertAlphabetically)
        self.setEditable(True)
        volume_list = ["0", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100"]

        self.addItems( volume_list )
        self.currentIndexChanged.connect( self.apply_volume )

    def apply_volume(self):
        volume = self.currentText()
        rospy.loginfo("set volume as: "+ str(volume))
        set_volume = rospy.ServiceProxy('/naoqi_driver/set_volume', SetAudioMasterVolume)
        req = SetAudioMasterVolumeRequest()
        req.master_volume.data=int(volume)
        res = set_volume(req)
        return res
