
机�?臂auto模式下的高层状态机
1.  STATE_TO_WAIT 
2.  STATE_ALIGN
3.  STATE_LOWER
4.  STATE_EXT
5.  STATE_LAUNCH
6.  STATE_BACK
7.  STATE_DONE 

#### STATE_TO_WAIT
这版由于�?��下拾取，所以不用�?机�?臂在底盘行进间的朝向了，直接统一朝着云台180度方向即�?�?
升高至目标高�?(函数同�?进间拾取的state_toTargetHight)，�?处�?9号桩�?3号桩做特殊�?理，

改为统一升到最�?

#### STATE_ALIGN
在�?驶至B1位置时候，云台旋转到KFS侧面法向方向�?//暂时弃�?
云台升到最高后旋转到�?先法平面法向

#### STATE_LOWER
判断PA点，云台�?��越过PA点后执�?下降到目标高度�?

#### STATE_EXT
执�?伸展，�?300ms后跳至下阶�?并执行回收�?

#### STATE_LAUNCH
执�?�?��到安全高度，并向底盘发送启动标志位

#### STATE_BACK
由于暂无存储机构，所以直接转到车尾�?即可

STATE_DONE
�?���?��来重�?��志位后放idle()，之后拾取两�?��至更多时候可以作为过渡态�?