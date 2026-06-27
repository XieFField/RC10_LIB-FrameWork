### 数据显示 下的内容

到时候需要根据主机发送的指令位切换显示的内容，
如果你觉得显示的内容太紧凑也可以做调整

模式枚举总：
	1. ALL_STOP
	2. SET_MODE
	3. MANUAL_CHASSIS
	4. MANUAL_ARM
	5. MANUAL_WEAPON_A
	6. MANUAL_WEAPON_B
	7. AUTO_NONE
	8. AUTO_ARM
	9. AUTO_WEAPON_A
	10. AUTO_WEAPON_B

显示内容一览总共有
	1. Locate: x:f32 , y:f32, yaw:f32  机器人坐标
	2. Target_KFS: kfs1:int8, kfs2:int8
	3. Target_Spear: Spear:int8
	4. Sucker: stauts:bool
	5. Claw:  stauts:bool
	6. JointArm: height:f32, strech:f32, rotate:f32, pitch:f32
	7. JointWeapon: height:f32, claw:f32, traverse:f32, wrist:f32
	8. R2CMD: enum1, enum2, enum3,enum4   暂时没定是什么指令，可能也没有4个这么多，你做个结构体里面塞四个枚举
	9. AutoStatus_ARM: enum （1.stop 2.execute1, 3.execute2, 4.over）
	10. AutoStatus_Weapon: enum(1.stop, 2.execute, 3.over)
	11. isArmCalibrate: status:bool
	12. isWeaponCalibrate: status:bool
