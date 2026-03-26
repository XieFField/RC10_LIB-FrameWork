
机械臂auto模式下的高层状态机
1.  STATE_TO_WAIT 
2.  STATE_ALIGN
3.  STATE_LOWER
4.  STATE_EXT
5.  STATE_LAUNCH
6.  STATE_BACK
7.  STATE_DONE 

#### STATE_TO_WAIT
这版由于是停下拾取，所以不用管机械臂在底盘行进间的朝向了，直接统一朝着云台180度方向即可。
升高至目标高度(函数同行进间拾取的state_toTargetHight)，此处对9号桩和3号桩做特殊处理，

#### STATE_ALIGN
在行驶至B1位置时候，云台旋转到KFS侧面法向方向。

#### STATE_LOWER
判断PA点，云台中心越过PA点后执行下降到目标高度。

#### STATE_EXT
执行伸展，读300ms后跳至下阶段并执行回收。

#### STATE_LAUNCH
执行抬升到安全高度，并向底盘发送启动标志位

#### STATE_BACK
直接转回180度位置 目前还没定下存储机构，暂时不考虑那么多

STATE_DONE
目前只用来重置标志位后放idle()，之后拾取两个甚至更多时候可以作为过渡态。