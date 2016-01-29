#!/usr/bin/python
#    Copyright 2007 John Kasunich and Jeff Epler
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


from vismach import *
import hal

c = hal.component("ra605gui")
c.newpin("joint1", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint2", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint3", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint4", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint5", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint6", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("grip", hal.HAL_FLOAT, hal.HAL_IN)
c.ready()


###################
# this stuff is the actual definition of the machine
# ideally it would be in a separate file from the code above
#

# gripper fingers
finger1 = CylinderZ(-10, 7.5, 50, 6)
finger2 = CylinderZ(-10, 7.5, 50, 6)
finger1 = HalRotate([finger1],c,"grip", 40,0,1,0)
finger2 = HalRotate([finger2],c,"grip",-40,0,1,0)
finger1 = Translate([finger1], 25,0.0,50)
finger2 = Translate([finger2],-25,0.0,50)
# "hand" - the part the fingers are attached to
# "tooltip" for backplot will be the origin of the hand for now
tooltip = Capture()
link6 = Collection([
	tooltip,
	Box(-50, -10, 10,  50, 10, 50),
	Box(-45, -45, 0.0, 45, 45, 10)])
# assembly fingers, and make it rotate
link6 = HalRotate([finger1,finger2,link6],c,"joint6",1,0,0,1)

# moving part of wrist joint
link5 = Collection([
	CylinderZ( 45, 50, 55, 50),
	CylinderX(-30, 40, 30, 40),
	Box(-27.5, -40, 0.0, 27.5, 40, 45)])
# move gripper to end of wrist and attach
link5 = Collection([
	link5,
	Translate([link6], 0, 0, 55)])
# make wrist bend
link5 = HalRotate([link5],c,"joint5",1,1,0,0)

# fixed part of wrist joint (rotates on end of arm)
link4 = Collection([
	CylinderX(-25, 35, -50, 35),
	CylinderX( 25, 35,  50, 35),
	Box(-45, -40, -50, -30, 40, 0.0),
	Box( 45, -40, -50,  30, 40, 0.0),
	Box(-45, -45, -70,  45, 45, -50)])
# attach wrist, move whole assembly forward so joint 4 is at origin
link4 = Translate([link4,link5], 0, 0, 70)
# make joint 4 rotate
link4 = HalRotate([link4],c,"joint4",1,0,0,1)

# next chunk
link3 = CylinderZ(0.0, 45, 338, 40)
# offset for A3(40)
link3 = Translate([link3], 0, -40, 0.0)
link3 = Collection([
	link3,
	CylinderX(-85, 57, -70, 63),
	CylinderX(-70, 65, 0, 65)])
# move link4 forward and attach
link3 = Collection([
	link3,
	Translate([link4],0.0, -40, 338)])
# move whole assembly over so joint 3 is at origin
link3 = Translate([link3], 0, 0.0, 0.0)
link3 = Rotate([link3], -90,1,0,0)
# make joint 3 rotate
link3 = HalRotate([link3],c,"joint3",1,1,0,0)

# elbow stuff
link2 = Collection([
	CylinderX(0, 55, 3, 55),
	CylinderX(3, 65, 80, 65),
	CylinderX(80, 63, 90, 57)])
# move elbow to end of upper arm
link2 = Translate([link2],0.0,0.0,340)
# rest of upper arm
link2 = Collection([
	link2,
	CylinderZ(340, 55, 0.0, 65),
	CylinderX(0, 75, 90, 75),
	CylinderX(90, 73, 105, 65)])
# move link 3 into place and attach
link2 = Collection([
	link2,
	Translate([link3], 0, 0.0, 340)])  # place link3 at relative position (x, y, z)
# move whole assembly over so joint 2 is at origin
link2 = Translate([link2], 0, 0, 0.0)
link2 = Rotate([link2], 90,-1,0,0)

# make joint 2 rotate
link2 = HalRotate([link2],c,"joint2",1,-1,0,0)

# shoulder stuff
link1 = Collection([
	CylinderX(-3, 65, 0, 65),
	CylinderX(-105, 75, -3, 75),
	CylinderX(-105, 73, -120, 65)])
# move link2 to end and attach
link1 = Collection([
	link1,
	Translate([link2], 0, 0, 0)])
# move whole assembly forward(y=30), so joint 1 is at origin
link1 = Translate([link1],0, 30, 0)
link1 = Rotate([link1], -90,0,0,1)
# make joint 1 rotate
link1 = HalRotate([link1],c,"joint1",1,0,0,1)  # th(1), x(0), y(0), z(1)

# stationary base
link0 = Collection([
	CylinderZ(295, 90, 375, 90),
	CylinderZ(9, 90, 355, 85),
	CylinderZ(0.00, 105, 10, 105)])
# move link1 to top and attach
link0 = Collection([
	link0,
	Translate([link1],0.0,0.0,375)])

# add a floor
floor = Box(-157,-157,-10,157,157,0.0)

work = Capture()

model = Collection([link0, floor, work])

main(model, tooltip, work, size=600)
