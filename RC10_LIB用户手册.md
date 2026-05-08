## RC10_LIB FrameWork鐢ㄦ埛鎵嬪唽

鐢ㄦ埛鎵嬪唽锛熶害鎴栬?鏄??鍒惰彍鐨勪竴鐜?.

RC10_LIB灏嗘彁渚涘ぇ閲忛?鍒惰彍锛屾棬鍦ㄨ?瀵瑰簳灞傞┍鍔ㄤ笉鐔熸倝鐨勭敤鎴蜂篃鑳界晠蹇?功鍐欏簲鐢ㄥ眰浠ｇ爜銆?
鑰屾湰鐢ㄦ埛鎵嬪唽涔熸槸棰勫埗鑿滅殑涓€鐜?紝鏃ㄥ湪璁╃敤鎴峰彲浠ユ洿蹇?笂鎵嬩娇鐢≧C10_LIB

**attention**: 杩欎唤鎵嬪唽寰堝ぇ绋嬪害鏄疉I鐢熸垚鐨勶紝绗旇€呭彧璐熻矗淇?敼鍏朵腑閮ㄥ垎锛岃嫢鍙戠幇鏈夌喊婕忥紝璇峰強鏃跺憡璇夋垜锛屼竾鍒嗘劅璋€€?

### 绋嬪簭涓?洰鍓嶆墽琛岀殑鍛藉悕瑙勮寖
1. 鍦ㄧ被涓?殑鍙橀噺缁熶竴甯? _ 鐨勫悗缂€锛屽?`rpm_`
2. 鍦ㄧ被涓?殑鎴愬憳浠ュ皬鍐欏瓧姣嶅紑澶?
3. 绫诲悕涓嶈?绾?皬鍐欏瓧姣嶅拰澶у啓瀛楁瘝
4. RC10_LIB搴撲腑鐨勫ご鏂囦欢涓庢簮鏂囦欢鍛藉悕闇€甯﹀垎鏀?殑鍓嶇紑锛屽?"Motor_","BSP_"

### 寮€鍙戝缓璁?
1. 澶氬啓娉ㄩ噴锛屽?鏋滄噿寰楀啓锛屽彲浠ュ儚鎴戜竴鏍风敤vscode鑷?甫鐨刟i琛ュ叏娉ㄩ噴
2. 褰撴偍鍦ㄥ紑鍙戞病鏈夊ご缁?椂鍊欙紝鍙?互鍥為【寮€鍙戞墜鍐?
3. 涓嶈?灏嗛潪API鍔犲叆RC10_LIB
4. 绂佹?涓€鍒囧姩鎬佸唴瀛樺垎閰?
5. 涓€鍒囧潗鏍囬噰鐢ㄥ彸鎵嬬郴锛屼笉绗﹀悎鐨勫氨鍙樻崲涓哄彸鎵嬬郴
6. 姝ゆ?鏋跺唴鐨勪竴鍒囨秹鍙婅?搴﹁?閫熷害鐨勯兘涓嶈?鐩存帴浣跨敤寮у害鍒?
7. 鎵€鏈夊叧浜庤?搴︾殑锛岄兘搴斿綋鍙樻崲涓篬0,360](鍖归厤PID涓?殑璁捐?)

### User
1. 鏈烘瀯鎺у埗绫绘斁鍦–ontrol
2. 璋冭瘯debug/demo绫绘斁鍦╠ebug
3. Setup鏄?姛鑳藉垵濮嬪寲銆佷互鍙婂姛鑳借繍琛岀殑鍦版柟

### RC10_LIB鐨勬牳蹇冭?璁″師鍒?
1. 涓ユ牸鍒嗗眰锛岃亴璐ｅ崟涓€
   妗嗘灦鍒嗕负纭?欢椹卞姩灞傘€佽?澶囧崗璁?眰銆佺畻娉曞眰鍜屽簲鐢ㄥ眰銆傚綋浣犳坊鍔犳柊鍔熻兘鏃讹紝蹇呴』鏄庣‘鍏跺綊灞炪€?

   纭?欢椹卞姩鍙?礋璐ｄ笌鐗╃悊鎬荤嚎閫氫俊銆?
   璁惧?鍗忚?鍙?礋璐ｈВ鏋愬拰鎵撳寘鐗瑰畾璁惧?鐨勬姤鏂囥€?
   绠楁硶鏄?函绮圭殑鏁板?宸ュ叿銆?
   搴旂敤鍙?礋璐ｄ笅杈鹃珮灞傛寚浠ゃ€? 鍘熷垯锛氫竴鑸?畻娉曞眰涓嶆秹鍙婁换浣曠‖浠惰?澶囥€佸熀灞傚彧鑳借皟鐢ㄥ熀灞傘€?

2. 淇′换鑷?姩鍖栬皟搴︼紝鍒嗙?璁＄畻涓庢墦鍖?
   1. 渚嬪?: fdCANbus 妗嗘灦鎻愪緵涓€涓?珮棰戠巼鐨勪腑澶?皟搴﹀櫒锛屽畠浼氳嚜鍔ㄨ皟鐢ㄦ墍鏈夋敞鍐岃?澶囩殑 update() 鍜? packCommand()銆?

   1. update(): 鍙?敤浜庤?绠椼€傛墽琛屽?PID绛夊懆鏈熸€х畻娉曪紝鏇存柊鍐呴儴鐘舵€併€?
   2. packCommand(): 鍙?敤浜庢墦鍖呫€傝?鍙? update() 鐨勮?绠楃粨鏋滐紝骞跺皢鍏剁粍瑁呮垚寰呭彂閫佺殑CAN鎶ユ枃銆?
   3. setTarget...(): 鍙?敤浜庢帴鏀舵寚浠ゃ€傝繖鏄?綘鐨勯┍鍔ㄦ彁渚涚粰搴旂敤灞傜殑鎺ュ彛锛岀敤浜庤?缃?珮绾х洰鏍囥€? 
   4. 鍘熷垯锛? 姘歌繙涓嶈?鍦? packCommand() 涓?繘琛岃?绠楋紝涔熶笉瑕佸湪 update() 涓?粍瑁呮姤鏂囥€傜浉淇¤皟搴﹀櫒浼氭寜姝ｇ‘鐨勯『搴忚皟鐢ㄥ畠浠?€?

3. 缁ф壙缁熶竴鎺ュ彛锛屽埄鐢ㄥ?鎬佸疄鐜扮壒寮傛€?
   妗嗘灦閫氳繃闈㈠悜鎺ュ彛缂栫▼瀹炵幇鎵╁睍鎬с€傛墍鏈夎?澶囬┍鍔ㄩ兘蹇呴』缁ф壙鑷?竴涓?叡鍚岀殑鍩虹被锛堝? Motor_Base锛夈€?

   缁熶竴绠＄悊: 璋冨害鍣ㄥ彧涓庡熀绫绘帴鍙ｄ氦浜掞紝瀹冧笉鍏冲績鍏蜂綋鏄?粈涔堣?澶囥€?
   铏氬嚱鏁板疄鐜板?鎬?: 浣跨敤 virtual 鍑芥暟锛堝? get_GearRatio()锛夋潵璁╂瘡涓?瓙绫绘彁渚涜嚜宸辩嫭鐗圭殑淇℃伅鎴栬?涓恒€? 鍘熷垯锛? 浣犵殑鏂拌?澶囬┍鍔ㄥ繀椤诲疄鐜板熀绫荤殑鎵€鏈夌函铏氬嚱鏁帮紝骞跺埄鐢ㄨ櫄鍑芥暟閲嶅啓锛坥verride锛夋潵瀹炵幇鍏剁壒瀹氬崗璁?拰鍔熻兘銆?

4. 鐢ㄦ埛浣跨敤鎺ュ彛鐨勭畝鍖?
   灏嗕竴鍒囩殑閲嶅?鎬у伐浣滈兘鍦ㄧ被鐨勫皝瑁呬腑瀹炵幇锛屼娇寰楃敤鎴峰湪寮€鍙戝簲鐢ㄥ眰鐨勬椂鍊欐棤闇€鍐欏お澶氬啑鏉傞噸澶嶇殑浠ｇ爜锛屾洿楂樻晥杩涜?寮€鍙戙€?

### BSP鍒嗘敮
#### FreeRTOS鐨勪娇鐢?
鍦╜BSP_RTOS.h`鏂囦欢涓?紝灏佽?浜嗗熀鏈?殑RTOS浣跨敤锛岀洰鍓嶆湁鍩烘湰鐨勪换鍔″拰闃熷垪銆?

1.  **RtosTask 浠诲姟灏佽?**
    `RtosTask` 绫绘彁渚涗簡涓ょ?浠诲姟妯″紡锛岄€氳繃鏋勯€犲嚱鏁扮殑 `period` 鍙傛暟鍖哄垎锛?
    *   **鍛ㄦ湡鎬т换鍔? (`period > 0`)**: 浠诲姟浼氫互 `period` 鎸囧畾鐨凾ick闂撮殧鑷?姩寰?幆鎵ц? `loop()` 鏂规硶銆傞€傜敤浜庨渶瑕佸浐瀹氶?鐜囪繍琛岀殑绠€鍗曢€昏緫銆?
        ```cpp
        class MyPeriodicTask : public RtosTask {
        public:
            MyPeriodicTask() : RtosTask("MyTask", 1000) {} // 1000ms鍛ㄦ湡
        protected:
            void loop() override 
            {
                // 杩欓噷鐨勪唬鐮佹瘡1000ms鎵ц?涓€娆?
            }
        };
        ```
    *   **浜嬩欢椹卞姩浠诲姟 (`period = 0`)**: 浠诲姟鍒涘缓鍚庝細鎵ц?涓€娆? `run()` 鏂规硶銆俙run()` 鏂规硶蹇呴』鍖呭惈涓€涓??寰?幆 `for(;;)` 鍜屼竴涓?樆濉炶皟鐢?紙濡? `vTaskDelay`, `xSemaphoreTake`锛夛紝鐢ㄤ簬绛夊緟澶栭儴浜嬩欢銆傞€傜敤浜庨渶瑕佽?鍔ㄨЕ鍙戠殑澶嶆潅浠诲姟锛屼緥濡侰AN鎬荤嚎鐨勮皟搴﹀拰鎺ユ敹浠诲姟銆?
        ```cpp
        class MyEventTask : public RtosTask {
        public:
            MyEventTask() : RtosTask("EventTask", 0) {} // 浜嬩欢椹卞姩
        protected:
            void run() override 
            {
                init(); //浼氳?鎵ц?
                for(;;) 
                {
                    // 绛夊緟淇″彿閲忔垨鍏朵粬浜嬩欢
                    xSemaphoreTake(mySemaphore, portMAX_DELAY); 
                    // 澶勭悊浜嬩欢...
                }
            }
        };
        ```

2.  **RtosQueue 闃熷垪灏佽?**
    杩欐槸涓€涓?ā鏉跨被锛屽彲浠ユ柟渚垮湴鍒涘缓鍜屼娇鐢ㄧ嚎绋嬪畨鍏ㄧ殑闃熷垪銆?
    ```cpp
    // 鍒涘缓涓€涓?兘瀹圭撼8涓猧nt鐨勯槦鍒?
    RtosQueue<int> myQueue(8);

    // 鍦ㄤ竴涓?换鍔′腑鍙戦€佹暟鎹?
    myQueue.send(123);

    // 鍦ㄥ彟涓€涓?换鍔′腑鎺ユ敹鏁版嵁
    int received_value;
    if (myQueue.recv(received_value, 100)) { // 绛夊緟100ms
        // 鎴愬姛鎺ユ敹鍒版暟鎹?
    }
    ```

### APP鍒嗘敮

#### APP_tool
宸ュ叿绫伙紝鎻愪緵濡? `constrain`锛堥檺骞咃級绛夐€氱敤鍑芥暟銆?

#### APP_debugTool
鎻愪緵璋冭瘯宸ュ叿锛屽?涓插彛鎵撳嵃鏁版嵁銆?

#### APP_PID
鎻愪緵浜嗕綅缃?紡鍜屽?閲忓紡涓ょ?PID鎺у埗鍣ㄣ€?

1.  **鏍稿績璁捐?**
    *   **浣嶇疆寮廝ID**: 閲囩敤浜嗘?褰㈢Н鍒嗐€佸井鍒嗗厛琛屻€佺Н鍒嗗垎绂荤瓑鏀硅繘绠楁硶锛岄€傜敤浜庡ぇ閮ㄥ垎闇€瑕佺簿纭?綅缃?帶鍒剁殑鍦烘櫙銆?
    *   **澧為噺寮廝ID**: 鍔犲叆浜嗗井鍒嗚窡韪?櫒(Track_D)锛岃兘鏈夋晥骞虫粦鐩?爣鍊肩殑闃惰穬鍙樺寲锛屽噺灏戠郴缁熼渿鑽★紝閫傜敤浜庨€熷害鎺у埗绛夊満鏅?€?
    *   **鍥哄畾閲囨牱鏃堕棿**: PID鎺у埗鍣ㄥ唴閮ㄧ殑 `dt` 浣跨敤鏃堕棿鎴虫柟寮忚?绠楋紝浣嗗畠澶ч儴鍒嗘椂鍊欑殑鍊兼槸涓?1ms銆傝繖鏄?竴涓?**鏍稿績璁捐?**锛屽畠寮轰緷璧栦簬璋冪敤 `pid_calc` 鐨? `update()` 鏂规硶琚?竴涓?簿纭?殑1kHz璋冨害鍣?紙濡? `fdCANbus::schedulerTaskbody`锛夋墍璋冪敤銆傚悗缁?細鑰冭檻鎶婃潹鍝ラ偅濂楃敤缂栫爜鍊艰?绠楁椂闂寸殑浠ｇ爜鎼?繃鏉ワ紝鍙?互璁ヾt鏇村姞绮剧‘銆?

2.  **鐢ㄦ埛璇ュ?浣曚娇鐢?紵**
    鍦ㄧ數鏈虹被锛堝? `M3508`锛夌殑 `pid_init` 鍑芥暟涓?垵濮嬪寲PID鍙傛暟锛岀劧鍚庡湪 `update` 鍑芥暟涓?皟鐢? `pid_calc` 鍗冲彲銆傜敤鎴锋棤闇€鍏冲績 `dt` 鐨勮?绠椼€?
    ```cpp
    // 鍦? M3508::update() 涓?
    case SPEED_CONTROL:
    {
        // target_rpm_ 鍜? this->rpm_ 閮芥槸杈撳嚭杞磋浆閫燂紝灏哄害缁熶竴
        target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
        break;
    }
    ```

    **濡傛灉浣犱娇鐢ㄧ殑鏄?綅缃?紡PID**
        浣嶇疆寮廝ID鍖呭惈浜嗕袱绉嶆ā寮忥細
            1. 绾挎€фā寮忥細姝ゆā寮忎笅锛岄€傚悎璺?▼寮忕殑PID
            2. 寰?幆妯″紡锛氭?妯″紡涓嬶紝閫傚悎浜戝彴寮忕殑PID锛岃寖鍥翠负[0,360];

#### APP_CoordConvert
`APP_CoordConvert` 鏄?竴涓?熀浜? `CMSIS-DSP` 搴撲紭鍖栫殑楂樻€ц兘鍧愭爣鍙樻崲宸ュ叿锛岀敤浜庡?鐞?2D鍜?3D绌洪棿涓?殑骞崇Щ鍜屾棆杞?€?

##### 鏍稿績鐗规€?
- **楂樻€ц兘**: 鎵€鏈夌煩闃佃繍绠楅兘鐢? `arm_math.h` 涓?殑鍑芥暟瀹屾垚锛屽厖鍒嗗埄鐢ㄧ‖浠跺姞閫熴€?
- **鏄撲簬浣跨敤**: 鎻愪緵浜? `HomogeneousTransform2D` 鍜? `HomogeneousTransform3D` 涓や釜绫伙紝鎺ュ彛娓呮櫚鐩磋?銆?
- **鍔熻兘瀹屽?**: 鏀?寔璁剧疆鍙樻崲銆佸簲鐢ㄥ彉鎹€€佺煩闃典箻娉曪紙鍙樻崲鍙犲姞锛夊拰姹傞€嗗彉鎹€€?

##### **銆愰噸瑕佹彁绀恒€?**
- **瑙掑害鍗曚綅**: 鎵€鏈夊嚱鏁扮殑瑙掑害鍙傛暟锛堝? `theta_rad`, `roll_rad`锛夐兘蹇呴』浣跨敤 **寮у害 (radians)** 浣滀负鍗曚綅銆?
- **鍛藉悕绌洪棿**: 鎵€鏈夌被鍜屽嚱鏁伴兘浣嶄簬 `geometry` 鍛藉悕绌洪棿涓嬨€?

##### 2D鍙樻崲浣跨敤绀轰緥

鍋囪?鏈変竴涓?紶鎰熷櫒瀹夎?鍦ㄦ満鍣ㄤ汉涓婏紝鍏跺潗鏍囩郴鐩稿?浜庢満鍣ㄤ汉涓?績鍧愭爣绯绘湁濡備笅鍏崇郴锛?
- 娌挎満鍣ㄤ汉X杞村钩绉讳簡 `0.2` 绫炽€?
- 娌挎満鍣ㄤ汉Y杞村钩绉讳簡 `0.1` 绫炽€?
- 閫嗘椂閽堟棆杞?簡 `45` 搴︺€?

鐜板湪锛屼紶鎰熷櫒妫€娴嬪埌浜嗕竴涓?湪鍏惰嚜韬?潗鏍囩郴涓嬬殑鐐? `(0.5, 0.0)`锛屾垜浠?兂鐭ラ亾杩欎釜鐐瑰湪鏈哄櫒浜轰腑蹇冨潗鏍囩郴涓嬬殑浣嶇疆銆?

```cpp
#include "APP_CoordConvert.h"
#include "arm_math.h" // For PI constant

// 浣跨敤鍛藉悕绌洪棿
using namespace geometry;

void transform_example_2d()
{
    // 1. 瀹氫箟涓€涓? Point2D 瀵硅薄鏉ユ弿杩颁粠浼犳劅鍣ㄥ埌鏈哄櫒浜轰腑蹇冪殑浣嶅Э
    //    骞崇Щ (0.2, 0.1)锛屾棆杞? 45 搴? (PI/4 寮у害)
    Point2D sensor_pose(0.2f, 0.1f, PI / 4.0f);

    // 2. 浣跨敤璇ヤ綅濮垮?璞″垱寤哄彉鎹㈢煩闃?
    HomogeneousTransform2D sensor_to_robot_tf(sensor_pose);

    // 3. 瀹氫箟鍦ㄤ紶鎰熷櫒鍧愭爣绯讳笅鐨勭偣
    Point2D point_in_sensor(0.5f, 0.0f);

    // 4. 搴旂敤鍙樻崲锛屽緱鍒板湪鏈哄櫒浜哄潗鏍囩郴涓嬬殑鐐?
    Point2D point_in_robot = sensor_to_robot_tf.apply(point_in_sensor);

    // point_in_robot.x 鍜? point_in_robot.y 灏辨槸鏈€缁堢粨鏋?
}
```

##### 3D鍙樻崲浣跨敤绀轰緥

鍋囪?鐩告満鍧愭爣绯荤浉瀵逛簬涓栫晫鍧愭爣绯诲钩绉讳簡 `(1.0, 2.0, 0.5)`锛屽苟涓旂粫Z杞存棆杞?簡90搴︺€?

```cpp
#include "APP_CoordConvert.h"
#include "arm_math.h"

using namespace geometry;

void transform_example_3d()
{
    // 1. 瀹氫箟涓€涓? Point3D 瀵硅薄鏉ユ弿杩颁粠鐩告満鍒颁笘鐣屽潗鏍囩郴鐨勪綅濮?
    //    骞崇Щ (1, 2, 0.5)锛岀粫Z杞?(yaw)鏃嬭浆90搴? (PI/2)
    Point3D camera_pose(1.0f, 2.0f, 0.5f, 0.0f, 0.0f, PI / 2.0f);

    // 2. 浣跨敤璇ヤ綅濮垮?璞″垱寤哄彉鎹㈢煩闃?
    HomogeneousTransform3D camera_to_world_tf(camera_pose);

    // 3. 瀹氫箟鍦ㄧ浉鏈哄潗鏍囩郴涓嬬殑涓€涓?偣
    Point3D point_in_camera(0.0f, 1.0f, 0.0f);

    // 4. 搴旂敤鍙樻崲锛屽緱鍒板湪涓栫晫鍧愭爣绯讳笅鐨勭偣
    Point3D point_in_world = camera_to_world_tf.apply(point_in_camera);

    // 5. 璁＄畻閫嗗彉鎹?紙浠庝笘鐣屽潗鏍囩郴鍒扮浉鏈哄潗鏍囩郴锛?
    HomogeneousTransform3D world_to_camera_tf = camera_to_world_tf.inverse();

    // 6. 浣跨敤閫嗗彉鎹㈠皢涓栫晫鍧愭爣绯讳笅鐨勭偣杞?崲鍥炵浉鏈哄潗鏍囩郴
    Point3D point_back_in_camera = world_to_camera_tf.apply(point_in_world);
    // 姝ゆ椂 point_back_in_camera 搴旇?绾︾瓑浜? point_in_camera
}
```

### Module鍒嗘敮
姝ゅ垎鏀?富瑕佸寘鍚?笌鐗瑰畾纭?欢妯″潡鐩稿叧鐨勪唬鐮侊紝渚嬪? `Module_Encoder.cpp`锛屽畠璐熻矗灏嗙紪鐮佸櫒鐨勫師濮嬪€硷紙濡?0-8191锛夎浆鎹?负杩炵画鐨勮?搴︼紙-鈭?, +鈭烇級鍜屽崟鍦堣?搴?0, 360]銆?

#### Chassis_Base 搴曠洏鍩虹被浣跨敤鎸囧崡

`Chassis_Base` 鏄?竴涓?敤浜庢瀯寤哄悇绉嶅簳鐩樿繍鍔ㄥ?妯″瀷鐨勫己澶у熀绫汇€傚畠閲囩敤C++妯℃澘鍜岄潰鍚戝?璞＄殑璁捐?锛屽疄鐜颁簡杩愬姩瀛﹁В绠椾笌鍏蜂綋鐢垫満椹卞姩鐨勫畬鍏ㄨВ鑰︺€?

##### 鏍稿績璁捐?

- **闈欐€佹硾鍨嬭?璁?**: 浣跨敤 `template <std::size_t WheelCount>`锛屼綘鍙?互鍦ㄧ紪璇戞椂灏辩‘瀹氬簳鐩樼殑杞?瓙鏁伴噺锛屾墍鏈夊唴瀛樺潎涓洪潤鎬佸垎閰嶏紝绗﹀悎宓屽叆寮忕郴缁熺殑楂樺彲闈犳€ц?姹傘€?
- **鑱岃矗鍒嗙?**: `Chassis_Base` 鍙?礋璐ｈ繍鍔ㄥ?璁＄畻銆傚畠璁＄畻鍑烘瘡涓?疆瀛愬簲璇ヨ揪鍒扮殑鐩?爣杞?€燂紙RPM锛夛紝鐒跺悗閫氳繃 `setTargetRPM()` 灏嗚繖涓?洰鏍囦紶閫掔粰宸叉敞鍐岀殑鐢垫満瀵硅薄銆傚疄闄呯殑鐢垫満PID闂?幆鎺у埗鍜孋AN鎶ユ枃鍙戦€佸垯鐢? `fdCANbus` 鐨勮皟搴﹀櫒鑷?姩瀹屾垚銆?
- **鍧愭爣绯荤?鐞?**: 鍐呯疆鏈哄櫒浜哄潗鏍囩郴鍜屼笘鐣屽潗鏍囩郴鐨勯€熷害绠＄悊銆備綘鍙?渶閫氳繃 `updateAngleData()` 鎻愪緵瀹炴椂鐨勫亸鑸??锛坹aw锛夛紝鍩虹被灏辫兘鑷?姩澶勭悊涓や釜鍧愭爣绯讳箣闂寸殑閫熷害杞?崲銆?
- **鐙?珛鐨勬洿鏂板惊鐜?**: `Chassis_Base` 鐨? `update()` 鏂规硶**涓嶄細**琚? `fdCANbus` 鑷?姩璋冪敤銆備綘闇€瑕佸湪鑷?繁鐨勬帶鍒朵换鍔′腑锛屼互浣犳湡鏈涚殑棰戠巼鏉ヨ皟鐢ㄥ畠銆?


GitHub Copilot

浠ヤ笅鍐呭?鍙?洿鎺ョ矘璐村埌鈥淩C10_LIB鐢ㄦ埛鎵嬪唽.md鈥濄€?

## Module_Air_joy 鑸?ā閬ユ帶 PPM 椹卞姩浣跨敤鎸囧崡

AirJoy 鏄?竴涓?熀浜? EXTI 涓?柇涓庡井绉掔骇鏃堕棿鎴崇殑 PPM 瑙ｇ爜鍣ㄣ€傚畠灏? 8 璺?埅妯￠€氶亾鑴夊?锛堢害 1000鈥?2000 us锛夎В鏋愪负鏄撶敤鐨勬暟鍊兼垚鍛橈紝渚涘簳鐩樸€佹満姊拌噦绛変笂灞傛ā鍧楃洿鎺ヨ?鍙栥€?

### 1. 鍔熻兘涓庝緷璧?
- 鍔熻兘锛歅PM 杈撳叆锛岃嚜鍔ㄨ瘑鍒?抚澶达紝瑙ｆ瀽 8 璺?€氶亾鍒版垚鍛樺彉閲忥細
  - 妯℃嫙閲忥細LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y锛堝崟浣嶏細寰??锛?950鈥?2050锛?
  - 寮€鍏抽噺锛歋WA, SWB, SWC, SWD锛堝悓鏍锋槸鑴夊?鍊硷紝甯歌?涓轰袱/涓夋。锛?
- 渚濊禆锛?
  - 寰??绾ф椂闂存埑鏈嶅姟锛欱SP_TimeStamp锛堥渶鍏堝垵濮嬪寲锛?
  - GPIO 澶栭儴涓?柇锛圗XTI锛?
  - HAL 搴擄紙STM32H7锛?

### 2. 纭?欢涓? CubeMX 閰嶇疆
- 灏? PPM 淇″彿鎺ュ叆涓€涓?敮鎸? EXTI 鐨? GPIO锛?3.3V 閫昏緫锛屽缓璁??閮ㄤ笂鎷?/涓嬫媺鎸夋帴鏀跺櫒瑕佹眰閰嶇疆锛夈€?
- CubeMX 閰嶇疆姝ラ?锛?
  1. 閫夋嫨鐢ㄤ簬 PPM 鐨? GPIO 寮曡剼锛屾ā寮忚?涓? External Interrupt Mode锛堜笂鍗囨部瑙﹀彂鍗冲彲锛夈€?
  2. 浣胯兘璇? EXTI 鐨? NVIC 涓?柇銆?
  3. 閫夋嫨涓€涓?畾鏃跺櫒渚? TimeStamp 浣跨敤锛堜换鎰忕ǔ瀹氭椂閽熸簮锛夛紝淇濇寔涓€鐩磋繍琛屻€?
  4. 纭??绯荤粺鏃堕挓宸叉?纭?厤缃?€?

### 3. 鍒濆?鍖栦笌鍥炶皟
- 鍒濆?鍖? TimeStamp锛堢ず渚嬶級锛?
    ````cpp
    #include "BSP_TimeStamp.h"
    extern TIM_HandleTypeDef htim2;

    void user_setup()
    {
        TimeStamp::getInstance().init(&htim2); // 璁╁井绉掕?鏃跺紑濮嬭窇
        // ... 鍏朵粬鍒濆?鍖? ...
    }
    ````

- EXTI 鍥炶皟锛堝簱宸插唴缃?ず渚嬶級锛?
  - Module_Air_joy.cpp 涓?凡瀹炵幇
    air_joy.data_update(GPIO_Pin, GPIO_PIN_8);
  - 濡備綘鐨? PPM 涓嶅湪 PIN_8锛岃?鎶婄?浜屼釜鍙傛暟鏀逛负瀹為檯浣跨敤鐨勯偅涓? GPIO_PIN_XX銆?
  - 濡傛灉浣犳墦绠楄嚜宸卞啓鍥炶皟锛屽彲鍙傝€冿細
    ````cpp
    #include "Module_Air_joy.h"

    extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        // 鍋囪? PPM 鎺ュ湪 PB6
        air_joy.data_update(GPIO_Pin, GPIO_PIN_6);
    }
    ````

### 4. 璇诲彇鏁版嵁涓庡綊涓€鍖?
- AirJoy 鎴愬憳鍙橀噺瀹炴椂鏇存柊锛堝湪姣忔敹鍒颁竴鏁村抚鍚庡啓鍏ワ級锛屽崟浣嶅潎涓哄井绉掞細
  - LEFT_X = PPM_buf[0]
  - LEFT_Y = PPM_buf[1]
  - RIGHT_X = PPM_buf[3]
  - RIGHT_Y = PPM_buf[2]
  - SWA = PPM_buf[4], SWB = PPM_buf[5], SWC = PPM_buf[6], SWD = PPM_buf[7]
- 鍏稿瀷褰掍竴鍖栨柟娉曪紙灏? 1000鈥?2000 us 鏄犲皠鍒? -1~+1 鎴? 0~1锛夛細
    ````cpp
    ```cpp
    static inline float ppm_to_norm_pm(const uint16_t us, uint16_t mid=1500, float span=500.0f)
    {
        // [-1, +1]锛?1500 涓轰腑鍊硷紝卤500us 涓烘弧閲忕▼
        return (static_cast<float>(us) - static_cast<float>(mid)) / span;
    }

    static inline float ppm_to_norm_01(const uint16_t us, uint16_t min_us=1000, uint16_t max_us=2000)
    {
        float u = (float)(us - min_us) / (float)(max_us - min_us);
        if(u < 0.f) u = 0.f;
        if(u > 1.f) u = 1.f;
        return u;
    }
    ````

    - 绀轰緥锛氬皢閫氶亾鏄犲皠涓哄簳鐩橀€熷害鎸囦护
    ````cpp
    ```cpp
    // 鍋囪?浣跨敤鍏ㄥ悜搴曠洏锛屽崟浣嶈嚜瀹氾紙渚嬪? m/s銆乺ad/s锛?
    float vx_cmd  = ppm_to_norm_pm(air_joy.LEFT_Y)  * 1.0f; // 鍓?/鍚?
    float vy_cmd  = ppm_to_norm_pm(air_joy.RIGHT_X) * 1.0f; // 宸?/鍙?
    float yaw_cmd = ppm_to_norm_pm(air_joy.LEFT_X)  * 2.0f; // 鏃嬭浆
    // 鎸夐渶闄愬箙鍚庨€佸叆 setRobotSpeed 鎴? setWorldSpeed
    ````

### 5. 涓庝换鍔″惊鐜?殑鍏崇郴
- AirJoy 閫氳繃 EXTI 涓?柇鎸夎竟娌块噰鏍峰苟璁＄畻鑴夊?锛屼笉闇€瑕佷綘鍦ㄤ换鍔￠噷涓撻棬鈥滄洿鏂扳€濄€?
- 寤鸿?浠ュ浐瀹氬懆鏈燂紙渚嬪? 10ms锛夎?鍙栨垚鍛樺彉閲忓苟鍋氬綊涓€鍖栵紝鍐嶄笅鍙戠粰搴曠洏/鎵ц?鍣ㄣ€?
- 濡傛灉闇€瑕佸垽鏂?€滄暟鎹?槸鍚︽柊椴溾€濓紝鍙?湪鐢ㄦ埛浠ｇ爜涓??褰曚笂娆′娇鐢ㄧ殑鍊兼垨鏃堕棿锛屽苟瀵? TimeStamp 鍙栧樊鍊煎仛瓒呮椂鍒ゅ畾锛堜緥濡? >50ms 鍒欒?涓洪仴鎺ф柇鑱旓紝杩涘叆瀹夊叏妯″紡锛夈€?

### 6. 閫氶亾/寮曡剼鑷?畾涔?
- 鏈€澶ч€氶亾鏁帮細榛樿? 8锛圡AX_CHANNELS=8锛夈€?
- 閫氶亾鏄犲皠锛氬湪 Module_Air_joy.cpp 涓?彲璋冩暣 PPM_buf 绱㈠紩鍒? LEFT/RIGHT/SW 鐨勬槧灏勩€?
- EXTI 寮曡剼锛氫慨鏀? HAL_GPIO_EXTI_Callback 涓?紶缁? data_update 鐨? GPIO_PIN_* 甯搁噺鍗冲彲銆?
- 鏃堕棿闃堝€硷細
  - 甯х粨鏉熼槇鍊? FRAME_END_MIN锛堥粯璁? 2100 us锛?
  - PWM_MIN/PWM_MAX锛堥粯璁? 950/2050 us锛?
  鏍规嵁浣犵殑鎺ユ敹鏈哄崗璁?€傚綋璋冩暣銆?

### 7. 甯歌?闂??
- 鏃犳暟鎹?洿鏂帮細
  - 纭?? TimeStamp 宸? init 涓斿湪璺戯紙寰??閫掑?锛夈€?
  - 纭?? EXTI 閰嶇疆鍒版?纭?紩鑴氥€佽Е鍙戞部銆丯VIC 宸蹭娇鑳姐€?
  - 纭?? HAL_GPIO_EXTI_Callback 涓?娇鐢ㄧ殑 GPIO_PIN_* 涓庡疄闄呬竴鑷淬€?
- 鎶栧姩/鏁板€艰烦鍙橈細
  - 绾块暱銆佸共鎵般€佷笂鎷?/涓嬫媺閰嶇疆涓嶅綋閮戒細瀵艰嚧閿欒?瑙﹀彂銆?
  - 鍙?湪椹卞姩鍐呭?鍔犵畝鍗曟护娉?紙褰撳墠瀹炵幇涓衡€滅洿鎺ラ噰鏍封€濓紝渚夸簬浣庡欢杩燂級銆?
- 鍙?湁閮ㄥ垎閫氶亾鏇存柊锛?
  - 妫€鏌ユ帴鏀舵満杈撳嚭鏄?惁涓? PPM锛堜笉鏄? SBUS/IBUS锛夈€?
  - 妫€鏌? MAX_CHANNELS 涓庝綘鐨勬帴鏀舵満閫氶亾鏁版槸鍚﹀尮閰嶃€?

浠ヤ笂鍗冲彲蹇?€熸妸 PPM 閬ユ帶鎺ュ叆浣犵殑搴旂敤銆傚缓璁?厛涓插彛鎵撳嵃鍥涗釜涓婚€氶亾鐨勫師濮? us 鍊硷紝纭??鑼冨洿涓庝腑鍊硷紝鍐嶅仛褰掍竴鍖栨槧灏勪笌鎺у埗鑱旇皟銆?

##### 濡備綍浣跨敤 `Chassis_Base`

###### 1. 鍒涘缓浣犵殑搴曠洏瀛愮被 (AI鐢熸垚锛屼笉鐢ㄥ敖淇?)

棣栧厛锛屼綘闇€瑕佸垱寤轰竴涓?户鎵胯嚜 `Chassis_Base` 鐨勫瓙绫伙紝骞跺疄鐜板叾绾?櫄鍑芥暟銆備互涓€涓?洓杞?害鍏嬬撼濮嗚疆搴曠洏涓轰緥锛?

**`Module_MecanumChassis.h`**
```cpp
#include "Module_ChassisBase.h"

class MecanumChassis : public Chassis_Base<4> {
public:
    // 鏋勯€犲嚱鏁帮細浼犲叆杞?瓙鍗婂緞銆佹渶澶?PM鍜屽簳鐩樼殑鍑犱綍鍙傛暟
    MecanumChassis(float wheel_radius, float max_wheel_rpm, float wheel_distance_x, float wheel_distance_y);

protected:
    // 銆愬繀椤汇€戝疄鐜拌繍鍔ㄥ?鏇存柊
    void updateKinematics() override;

    // 銆愬繀椤汇€戝疄鐜伴€嗚В锛氫粠鏈哄櫒浜洪€熷害璁＄畻杞?€?
    void inverseKinematics(const Robot_Twist& twist) override;

    // 銆愬繀椤汇€戝疄鐜版?瑙ｏ細浠庤疆閫熻?绠楁満鍣ㄤ汉閫熷害
    void forwardKinematics() override;

private:
    // 楹﹁疆搴曠洏鐨勫嚑浣曞弬鏁?
    const float wheel_distance_x_; // 杞?瓙鍦╔鏂瑰悜涓婄殑鍗婇棿璺?
    const float wheel_distance_y_; // 杞?瓙鍦╕鏂瑰悜涓婄殑鍗婇棿璺?
};
```

**`Module_MecanumChassis.cpp`**
```cpp
#include "Module_MecanumChassis.h"

// 鏋勯€犲嚱鏁?
MecanumChassis::MecanumChassis(float wheel_radius, float max_wheel_rpm, float wheel_distance_x, float wheel_distance_y)
    : Chassis_Base<4>(wheel_radius, max_wheel_rpm),
      wheel_distance_x_(wheel_distance_x),
      wheel_distance_y_(wheel_distance_y)
{}

// 杩愬姩瀛︽洿鏂帮細鍏堥€嗚В锛屽啀姝ｈВ
void MecanumChassis::updateKinematics() {
    inverseKinematics(this->robot_twist_); // 浣跨敤缁忚繃鏂滃潯澶勭悊鍚庣殑褰撳墠閫熷害杩涜?閫嗚В
    forwardKinematics();                   // 鏍规嵁瀹為檯杞?€熷弽棣堬紙濡傛灉闇€瑕侊級杩涜?姝ｈВ
}

// 閫嗚В瀹炵幇
void MecanumChassis::inverseKinematics(const Robot_Twist& twist) {
    const float lx_plus_ly = wheel_distance_x_ + wheel_distance_y_;
    const float rad_per_s_to_rpm = 60.0f / (2.0f * PI);

    // 楹﹀厠绾冲?杞?€嗚В鍏?紡
    float wheel_speed_rad_s[4];
    wheel_speed_rad_s[0] = (twist.vx - twist.vy - twist.yaw_rate * lx_plus_ly) / wheel_radius_;
    wheel_speed_rad_s[1] = (twist.vx + twist.vy + twist.yaw_rate * lx_plus_ly) / wheel_radius_;
    wheel_speed_rad_s[2] = (twist.vx + twist.vy - twist.yaw_rate * lx_plus_ly) / wheel_radius_;
    wheel_speed_rad_s[3] = (twist.vx - twist.vy + twist.yaw_rate * lx_plus_ly) / wheel_radius_;

    // 灏嗚?绠楀嚭鐨勮?閫熷害(rad/s)杞?崲涓篟PM锛屽苟瀛樺叆鐩?爣鏁扮粍
    for (int i = 0; i < 4; ++i) {
        this->wheele_target_rpm_[i] = wheel_speed_rad_s[i] * rad_per_s_to_rpm;
    }
}

// 姝ｈВ瀹炵幇 (绀轰緥锛屽疄闄呭彲鑳介渶瑕佷粠鐢垫満鑾峰彇鐪熷疄閫熷害)
void MecanumChassis::forwardKinematics() {
    // 杩欓噷浠呬负绀轰緥锛屽疄闄呭簲鐢ㄤ腑浣犲彲鑳介渶瑕佷粠 wheels_[i]->getRPM() 鑾峰彇鐪熷疄杞?€熸潵璁＄畻
    // 姝ゅ?鏆傛椂鐣欑┖鎴栧熀浜庣洰鏍囬€熷害杩涜?浼扮畻
}
```

###### 2. 鍦ㄥ簲鐢ㄥ眰闆嗘垚

鍦ㄤ綘鐨? `user_setup` 鍜屾帶鍒朵换鍔′腑锛屽皢鎵€鏈夐儴鍒嗙粍鍚堣捣鏉ャ€?

```cpp
/* user_setup.cpp 鎴? main.cpp */
#include "Module_MecanumChassis.h"
#include "Motor_DJI.h"
#include "BSP_fdCAN_Driver.h"
#include "BSP_IMU.h" // 鍋囪?浣犳湁涓€涓狪MU妯″潡

// --- 鍏ㄥ眬瀵硅薄瀹氫箟 ---
// 銆愪慨鏀广€戦€氳繃 getInstance 鑾峰彇 CAN 鎬荤嚎鐨勫敮涓€瀹炰緥鎸囬拡
fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1, 1);

// 銆愪繚鎸佷笉鍙樸€戦潤鎬佸垱寤虹數鏈哄拰搴曠洏瀵硅薄
M3508 wheel_motors[4] = { M3508(1, CAN1_Bus), M3508(2, CAN1_Bus), M3508(3, CAN1_Bus), M3508(4, CAN1_Bus) };
DJI_Group DJI_Group_1(0x200, CAN1_Bus);
MecanumChassis my_chassis(0.076f, 450.0f, 0.2f, 0.25f); // 杞?崐寰?, 鏈€澶?PM, x闂磋窛, y闂磋窛
IMU_Class my_imu; // 鍋囪?鐨処MU瀵硅薄

// --- 鍒濆?鍖栧嚱鏁? ---
void user_setup() {
    // 1. 鍒濆?鍖栫數鏈哄拰PID
    for (int i = 0; i < 4; ++i) {
        wheel_motors[i].pid_init(/* ... */);
        DJI_Group_1.addMotor(&wheel_motors[i]);
        // 銆愪慨鏀广€戞敞鍐岀數鏈烘椂锛屼娇鐢? CAN1_Bus 鎸囬拡
        CAN1_Bus->registerMotor(&wheel_motors[i]);
    }
    CAN1_Bus->registerMotor(&DJI_Group_1);
    CAN1_Bus->init();

    // 2. 娉ㄥ唽杞?瓙鍒版満绠辨ā鍨?
    // 娉ㄦ剰杞?瓙椤哄簭瑕佷笌浣犵殑杩愬姩瀛︽ā鍨嬩竴鑷?
    my_chassis.registerWheelMotor(0, &wheel_motors[0]); // 鍙冲墠杞?
    my_chassis.registerWheelMotor(1, &wheel_motors[1]); // 宸﹀墠杞?
    my_chassis.registerWheelMotor(2, &wheel_motors[2]); // 宸﹀悗杞?
    my_chassis.registerWheelMotor(3, &wheel_motors[3]); // 鍙冲悗杞?

    // 3. 閰嶇疆鍔犻€熷害闄愬埗 (鍙?€?)
    my_chassis.reset_AccLimitStatus(true); // 鍚?敤
    my_chassis.reset_AccValue(1.0f);       // 1.0 m/s^2
}

// --- 鎺у埗浠诲姟 ---
class ChassisControlTask : public RtosTask {
public:
    ChassisControlTask() : RtosTask("ChassisTask", 10) {} // 10ms鍛ㄦ湡, 100Hz

protected:
    void loop() override {
        // 1. 浠庨仴鎺у櫒鎴栦笂浣嶆満鑾峰彇鐩?爣閫熷害
        Robot_Twist target_speed;
        target_speed.vx = remote.getChannel(2); // 鍋囪?浠庨仴鎺у櫒鑾峰彇鍓嶈繘閫熷害
        target_speed.vy = remote.getChannel(3); // 鍋囪?浠庨仴鎺у櫒鑾峰彇骞崇Щ閫熷害
        target_speed.yaw_rate = remote.getChannel(0); // 鍋囪?浠庨仴鎺у櫒鑾峰彇鏃嬭浆閫熷害

        // 2. 浠嶪MU鑾峰彇褰撳墠濮挎€?
        Angle_Twist current_angle = my_imu.getAngle();
        my_chassis.updateAngleData(current_angle);

        // 3. 璁剧疆鐩?爣閫熷害鍒版満绠辨ā鍨? (浣跨敤涓栫晫鍧愭爣绯?)
        my_chassis.setWorldSpeed(target_speed);

        // 4. 銆愭牳蹇冦€戞洿鏂版満绠辨ā鍨?
        // 杩欎細鎵ц?杩愬姩瀛﹁В绠楋紝骞跺皢鐩?爣RPM璁剧疆缁欑數鏈?
        my_chassis.update();
    }
};
```

閫氳繃浠ヤ笂姝ラ?锛屼綘灏辨垚鍔熷湴灏嗕竴涓?害鍏嬬撼濮嗚疆搴曠洏闆嗘垚鍒颁簡RC10_LIB妗嗘灦涓?€俙Chassis_Base` 璐熻矗浜嗗?鏉傜殑杩愬姩瀛﹁?绠楀拰鍧愭爣鍙樻崲锛岃€? `fdCANbus` 鍒欏湪鍚庡彴榛橀粯鍦颁繚璇佷簡鎵€鏈夌數鏈篜ID鐨勭簿纭?墽琛屻€備綘鐨勬帶鍒朵换鍔″彧闇€瑕佸叧娉ㄢ€滄垜鎯宠?搴曠洏浠ヤ粈涔堥€熷害绉诲姩鈥濊繖涓€楂樺眰閫昏緫銆?

---

### fdCANbus濡備綍宸ヤ綔鐨勶紵

`fdCANbus` 鏄?暣涓?數鏈烘帶鍒跺簱鐨勭?缁忎腑鏋€€傚畠璐熻矗澶勭悊搴曞眰鐨凜AN閫氫俊锛屽苟浠ョ簿纭?殑棰戠巼鑷?姩璋冨害鎵€鏈夌數鏈烘帶鍒朵换鍔★紝灏嗙敤鎴蜂粠绻佺悙鐨勫疄鏃舵帶鍒跺拰纭?欢浜や簰涓?В鏀惧嚭鏉ャ€?

#### 鏍稿績缁勪欢涓庡伐浣滄祦绋?

`fdCANbus` 鍐呴儴涓昏?鐢变袱涓?苟琛岀殑RTOS浠诲姟椹卞姩锛?

1.  **鎺ユ敹浠诲姟 (`rxTask_`)**:
    *   **宸ヤ綔**: 杩欐槸涓€涓?簨浠堕┍鍔ㄧ殑浠诲姟锛屽畠姘镐箙闃诲?骞剁瓑寰? `rxQueue_` 闃熷垪涓?殑鏂版秷鎭?€?
    *   **鏁版嵁娴?**:
        1.  褰揅AN纭?欢鎺ユ敹鍒颁竴涓?暟鎹?抚锛宍HAL_FDCAN_RxFifo0Callback` 涓?柇鏈嶅姟绋嬪簭锛圛SR锛夎?瑙﹀彂銆?
        2.  ISR璋冪敤 `fdcan_global_rx_isr`锛岃?鍑芥暟浠庣‖浠剁紦鍐插尯璇诲彇鍘熷?CAN鎶ユ枃銆?
        3.  鍘熷?鎶ユ枃琚?皝瑁呮垚 `CanFrame` 瀵硅薄锛屽苟琚?珛鍗虫帹鍏? `rxQueue_` 闃熷垪銆?
        4.  `rxTask_` 琚?敜閱掞紝浠庨槦鍒椾腑鍙栧嚭 `CanFrame`銆?
        5.  `rxTask_` 閬嶅巻鎵€鏈夊凡娉ㄥ唽鐨勭數鏈猴紙`motorList_`锛夛紝璋冪敤姣忎釜鐢垫満鐨? `matchesFrame()` 鏂规硶鏉ュ?鎵捐?鎶ユ枃鐨勨€滀富浜衡€濄€?
        6.  涓€鏃︽壘鍒板尮閰嶇殑鐢垫満锛屽氨璋冪敤鍏? `updateFeedback()` 鏂规硶锛屽皢鎶ユ枃浜ょ敱鐢垫満鑷??瑙ｆ瀽銆?

2.  **璋冨害浠诲姟 (`schedulerTask_`)**:
    *   **宸ヤ綔**: 杩欐槸涓€涓?珮浼樺厛绾х殑銆佺敱瀹氭椂鍣ㄧ簿纭?Е鍙戠殑鍛ㄦ湡鎬т换鍔★紝棰戠巼涓?1kHz銆?
    *   **鏁版嵁娴?**:
        1.  涓€涓?1kHz鐨勭‖浠跺畾鏃跺櫒涓?柇瑙﹀彂 `fdcan_global_scheduler_tick_isr()`銆?
        2.  璇?SR閲婃斁锛圙ive锛変竴涓?悕涓? `schedSem_` 鐨勪俊鍙烽噺锛岀劧鍚庣珛鍗抽€€鍑恒€?
        3.  `schedulerTask_` 鍦ㄥ惎鍔ㄥ悗灏变竴鐩撮樆濉炵瓑寰咃紙Take锛夎繖涓?俊鍙烽噺銆備竴鏃﹁幏鍙栧埌淇″彿閲忥紝瀹冨氨浼氳?鍞ら啋銆?
        4.  **鏇存柊**: 浠诲姟棣栧厛閬嶅巻鎵€鏈夋敞鍐岀殑鐢垫満锛堟垨鐢垫満缁勶級锛屽苟璋冪敤瀹冧滑鐨? `update()` 鏂规硶銆傝繖浼氳Е鍙慞ID璁＄畻绛夋帶鍒堕€昏緫銆?
        5.  **鎵撳寘**: 鎺ョ潃锛屼换鍔″啀娆￠亶鍘嗘墍鏈夊?璞★紝璋冪敤 `packCommand()` 鏂规硶鏉ユ敹闆嗛渶瑕佸彂閫佺殑CAN鎸囦护甯?,姝ゅ?鍒╃敤`packCommand()`鐨勮繑鍥炲€艰?褰曢渶瑕佸彂閫佸嚑甯?AN銆?
        6.  **鍙戦€?**: 鏈€鍚庯紝浠诲姟灏嗘墍鏈夋敹闆嗗埌鐨勬寚浠ゅ抚閫氳繃 `sendFrame()` 鏂规硶鍙戦€佸嚭鍘汇€俙sendFrame` 鍐呴儴浣跨敤浜掓枼閿? `tx_mutex_` 鏉ョ‘淇濆?浠诲姟璁块棶CAN纭?欢鐨勭嚎绋嬪畨鍏ㄣ€?
        7.  瀹屾垚涓€杞?皟搴﹀悗锛宍schedulerTask_` 杩斿洖寰?幆鐨勫紑濮嬶紝鍐嶆?闃诲?绛夊緟涓嬩竴娆＄殑淇″彿閲忥紝浠庤€屽疄鐜扮簿纭?殑1ms鍛ㄦ湡銆?

#### 鍏抽敭璁捐?鍐崇瓥

*   **涓?柇鏈嶅姟绋嬪簭锛圛SR锛夋渶灏忓寲**: ISR鍙?仛鏈€灏戠殑宸ヤ綔鈥斺€旇?鍙栨暟鎹?苟灏嗗叾鎺ㄥ叆闃熷垪銆傛墍鏈夎€楁椂鐨勬搷浣滐紙濡傞亶鍘嗐€佸尮閰嶃€佽В鏋愶級閮借浆绉诲埌浼樺厛绾ц緝浣庣殑 `rxTask_` 涓?墽琛岋紝杩欑‘淇濅簡绯荤粺鐨勫疄鏃跺搷搴旇兘鍔涖€?
*   **鍙戦€佷笌鎺ユ敹鍒嗙?**: 鎺ユ敹鏄?畬鍏ㄥ紓姝ュ拰浜嬩欢椹卞姩鐨勶紝鑰屽彂閫佸垯鏄?悓姝ュ拰鍛ㄦ湡鎬х殑銆傝繖绉嶈?璁＄?鍚堟帶鍒剁郴缁熺殑鍏稿瀷妯″紡锛氭寔缁?帴鏀跺弽棣堬紝骞朵互鍥哄畾鐨勯?鐜囪緭鍑烘帶鍒舵寚浠ゃ€?
*   **鍏ㄥ眬涓?柇璺?敱**: 閫氳繃涓€涓?叏灞€鐨? `g_fdcan_bus_map` 鏁扮粍锛屽彲浠ュ皢鏉ヨ嚜HAL搴撶殑銆佷笉鍖哄垎鍏蜂綋鎬荤嚎鐨凜椋庢牸涓?柇鍥炶皟锛岀簿纭?湴璺?敱鍒板?搴旂殑 `fdCANbus` C++瀵硅薄瀹炰緥涓娿€傝繖浣垮緱浠ｇ爜鍙?互杞绘澗鏀?寔澶氫釜CAN鎬荤嚎銆?
*   **绾跨▼瀹夊叏**: 閫氳繃浣跨敤RTOS闃熷垪锛坄RtosQueue`锛夊拰浜掓枼閿侊紙`tx_mutex_`锛夛紝`fdCANbus` 纭?繚浜嗗湪澶氫换鍔＄幆澧冧笅鏁版嵁浜ゆ崲鍜岀‖浠惰?闂?殑瀹夊叏鎬с€?

### 鐢垫満搴撴牳蹇冭?璁′笌浣跨敤鎸囧崡

鏈?寚鍗楀皢寮曞?浣犲畬鎴愪粠纭?欢鍒濆?鍖栧埌鍦? RTOS 浠诲姟涓?帶鍒剁數鏈虹殑瀹屾暣娴佺▼銆?

#### 鏍稿績璁捐?鎬濇兂

1.  **鏁版嵁杞?崲鍓嶇疆**: 鍦? `DJI_Motor::updateFeedback` 鍑芥暟涓?紝浠嶤AN鎬荤嚎鎺ユ敹鍒扮殑**鐢垫満杞?瓙鍘熷?鏁版嵁**锛堣浆閫熴€佺紪鐮佸櫒鍊硷級浼?**绔嬪嵆**閫氳繃铏氬嚱鏁? `get_GearRatio()` 鑾峰彇姝ｇ‘鐨勫噺閫熸瘮锛屽苟琚?浆鎹?负**鍑忛€熷悗鐨勮緭鍑鸿酱鏁版嵁**銆?

2.  **鍐呴儴鐘舵€佺粺涓€**: 杞?崲瀹屾垚鍚庯紝鎵€鏈夊瓨鍌ㄥ湪鍩虹被 `Motor_Base` 涓?殑鎴愬憳鍙橀噺锛坄rpm_`, `angle_`, `totalAngle_`锛夌殑鍚?箟閮界粺涓€涓?**杈撳嚭杞寸殑鐘舵€?**銆?

3.  **鎺у埗涓庡弽棣堝昂搴︾粺涓€**: PID鎺у埗鐜?矾锛堝湪 `update()` 鏂规硶涓?級鐨?**鐩?爣鍊?**锛堝? `target_rpm_`锛夊拰**鍙嶉?鍊?**锛堝? `this->rpm_`锛夐兘鍩轰簬**杈撳嚭杞寸殑灏哄害**杩涜?璁＄畻锛屼繚璇佷簡鎺у埗鐨勬?纭?€с€?

4.  **璋冨害鑷?姩鍖?**: 浣? **涓嶉渶瑕?** 鎵嬪姩璋冪敤 PID 璁＄畻鎴? CAN 鍙戦€佸嚱鏁般€俙fdCANbus` 鍐呴儴鐨? `schedulerTask` 浼氫互 1kHz 鐨勯?鐜囪嚜鍔ㄥ畬鎴愭墍鏈夊凡娉ㄥ唽鐢垫満锛堟垨鐢垫満缁勶級鐨? `update()` 鍜? `packCommand()` 璋冪敤銆?

5.  **鐢ㄦ埛鑱岃矗**: 浣犵殑宸ヤ綔闈炲父绠€鍗曪紝鍙?渶鍦ㄤ竴涓?嫭绔嬬殑鎺у埗浠诲姟涓?紝鏍规嵁闇€瑕佽皟鐢? `setTargetRPM()`, `setTargetAngle()` 绛夊嚱鏁版潵璁惧畾**杈撳嚭杞寸殑鐩?爣鍊?**鍗冲彲銆?

#### 绗?竴姝ワ細绯荤粺鍒濆?鍖?

鎵€鏈夌‖浠跺拰瀵硅薄鐨勫垵濮嬪寲閮藉簲璇ュ湪鍚?姩 RTOS 璋冨害鍣? (`osKernelStart()`) 涔嬪墠瀹屾垚銆傛帹鑽愬湪 `main.cpp` 鐨? `USER CODE BEGIN 2` 鍜? `USER CODE END 2` 涔嬮棿锛屾垨鑰呬竴涓?笓闂ㄧ殑 `user_setup.cpp` 鏂囦欢涓?繘琛屻€?

```cpp
/* main.cpp 鎴? user_setup.cpp */

#include "BSP_fdCAN_Driver.h"
#include "Motor_DJI.h"

// 1. 瀹氫箟鍏ㄥ眬瀵硅薄
// 銆愪慨鏀广€戜笉鍐嶇洿鎺ュ畾涔? fdCANbus 瀵硅薄锛岃€屾槸閫氳繃 getInstance 鑾峰彇鍏跺敮涓€瀹炰緥鐨勬寚閽?
fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1, 1);

// 銆愪繚鎸佷笉鍙樸€戦潤鎬佸垱寤虹數鏈哄拰鐢垫満缁勫?璞★紝骞跺皢 CAN1_Bus 鎸囬拡浼犻€掔粰瀹冧滑
M3508 m3508_1(1, CAN1_Bus);     // M3508鐢垫満, ID涓?1
DJI_Group DJI_Group_1(0x200, CAN1_Bus); // DJI鐢垫満缁?, 鍙戦€両D涓?0x200

// 2. 鍒涘缓涓€涓?垵濮嬪寲鍑芥暟
void user_setup()
{
    // --- PID鍙傛暟閰嶇疆 ---锛圓I鐢熸垚鐨勶紝骞堕潪閫氱敤鍙傛暟锛?
    PID_Param_Config speed_pid_params = 
    {
        .kp = 10.0f, .ki = 0.5f, .kd = 0.0f,
        .I_Outlimit = 5000.0f, .isIOutlimit = true,
        .output_limit = 16000.0f, .deadband = 0.0f
    };
    PID_Param_Config angle_pid_params = 
    {
        .kp = 0.5f, .ki = 0.0f, .kd = 0.0f,
        .I_Outlimit = 100.0f, .isIOutlimit = true,
        .output_limit = 500.0f, .deadband = 0.0f
    };
    m3508_1.pid_init(speed_pid_params, 0.0f, angle_pid_params, 30.0f);

    // --- 娉ㄥ唽涓庨厤缃? ---
    // 灏嗙數鏈烘坊鍔犲埌鐢垫満缁?
    DJI_Group_1.addMotor(&m3508_1);
    // 浣犲彲浠ョ户缁?坊鍔犳洿澶氱數鏈哄埌杩欎釜缁?...
    // DJI_Group_1.addMotor(&another_motor);

    // 銆愰噸瑕併€戝皢鐢垫満鏈?韩鍜岀數鏈虹粍閮芥敞鍐屽埌CAN鎬荤嚎
    // 銆愪慨鏀广€戦€氳繃 CAN1_Bus 鎸囬拡璋冪敤 registerMotor
    // 1. 娉ㄥ唽鐢垫満鏈?韩锛屼娇鍏惰兘鎺ユ敹鍙嶉?鎶ユ枃骞舵洿鏂扮姸鎬?
    CAN1_Bus->registerMotor(&m3508_1);
    // 2. 娉ㄥ唽鐢垫満缁勶紝浣垮叾鑳借?璋冨害鍣ㄨ皟鐢? packCommand() 鏉ユ墦鍖呭彂閫佺數娴佹寚浠?
    CAN1_Bus->registerMotor(&DJI_Group_1);

    // --- 鍚?姩鎬荤嚎 ---
    // 銆愪慨鏀广€戦€氳繃 CAN1_Bus 鎸囬拡璋冪敤 init
    // 杩欎細鍚?姩CAN鐨勬帴鏀朵腑鏂?拰1kHz鐨勮皟搴︿换鍔?
    CAN1_Bus->init();
}

// 鍦? main() 鍑芥暟涓?皟鐢?
int main(void)
{
    // ... HAL_Init(), SystemClock_Config(), MX_GPIO_Init(), MX_FDCAN1_Init() ...
    
    user_setup(); // 璋冪敤鎴戜滑鐨勫垵濮嬪寲鍑芥暟
    
    osKernelInitialize();
    // ... 鍒涘缓鍏朵粬鐢ㄦ埛浠诲姟 ...
    osKernelStart();
    
    // ...
}
```

#### 濡傛灉浣犳兂鎷撳睍鐢垫満锛?
鍋囪?浣犺?娣诲姞涓€涓?潪DJI鐨勩€佹湁鑷?繁鐙?壒CAN鍗忚?鐨勭數鏈猴紝渚嬪? MyMotor銆?

1. 鍒涘缓 `Motor_MyMotor.h`

```cpp
#include "Motor_Base.h"
#include "APP_PID.h" // 濡傛灉闇€瑕丳ID

class MyMotor : public Motor_Base {
public:
    // 1. 鏋勯€犲嚱鏁帮細璋冪敤鍩虹被鏋勯€犲嚱鏁?
    MyMotor(uint32_t id, fdCANbus* bus) 
        : Motor_Base(id, false, bus) // 鍋囪?浣跨敤鏍囧噯甯?
    {
        // 鍒濆?鍖栬?鐢垫満鐨勭?鏈夋垚鍛?
    }

    // 2. 銆愬繀椤汇€戣?鐩? packCommand
    //    鏍规嵁 target_current_ 绛夌洰鏍囧€硷紝鎵撳寘鎴愯?鐢垫満鐨凜AN甯?
    //    姝ゅ?鐨勮繑鍥炲€煎姟蹇呭疄鐜帮紝鍚﹀垯浼氳?fdCANbus妫€娴嬫€荤嚎涓奀AN甯ф暟閲忓紓甯革紝瀵艰嚧鍙戦€佷涪鍖呫€?
    std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override;

    // 3. 銆愬繀椤汇€戣?鐩? updateFeedback
    //    瑙ｆ瀽鏀跺埌鐨凜AN甯э紝鏇存柊 rpm_, angle_ 绛夋垚鍛樺彉閲?
    void updateFeedback(const CanFrame& cf) override;

    // 4. 銆愬繀椤汇€戣?鐩? matchesFrame
    //    鍒ゆ柇鏀跺埌鐨凜AN甯ф槸鍚﹀睘浜庤繖涓?數鏈?
    bool matchesFrame(const CanFrame& cf) const override;

    // 5. 銆愬繀椤汇€戣?鐩? get_GearRatio
    //    杩斿洖璇ョ數鏈虹殑鐪熷疄鍑忛€熸瘮
    float get_GearRatio() const override { return 27.0f; } // 鍋囪?鍑忛€熸瘮鏄?27

    // 6. 瀹炵幇 update 鏂规硶锛岀敤浜庢墽琛孭ID璁＄畻
    void update() override;

    // 7. 瀹炵幇 setTarget... 绛夋帶鍒舵帴鍙?
    void setTargetRPM(float rpm_set) override;

private:
    // 璇ョ數鏈虹殑绉佹湁鎴愬憳锛屽?PID鎺у埗鍣?
    PID_Incremental speed_pid_;
};
```

2. 鍦? `Motor_MyMotor.cpp` 涓?疄鐜板姛鑳?
```cpp
#include "Motor_MyMotor.h"

std::size_t MyMotor::packCommand(CanFrame outFrames[], std::size_t maxFrames) {
    // ... 鏍规嵁 this->target_current_ 鎵撳寘CAN甯? ...
    // outFrames[0].ID = 0x123;
    // outFrames[0].data[0] = ...;
    return 1; // 杩斿洖鎵撳寘鐨勫抚鏁?
}

void MyMotor::updateFeedback(const CanFrame& cf) {
    // ... 瑙ｆ瀽 cf.data ...
    // float raw_rpm = ...;
    // this->rpm_ = raw_rpm / get_GearRatio(); // 杞?崲涓鸿緭鍑鸿酱杞?€?
}

bool MyMotor::matchesFrame(const CanFrame& cf) const {
    // 鍒ゆ柇閫昏緫锛屼緥濡傦細
    return (cf.ID == (0x200 + this->motor_id_));
}

void MyMotor::update() {
    // ... 璋冪敤PID璁＄畻 ...
    // target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
}

void MyMotor::setTargetRPM(float rpm_set) {
    // ... 璁剧疆鐩?爣鍊? ...
    this->target_rpm_ = rpm_set;
}
```
3. 鍦ㄥ簲鐢ㄥ眰浣跨敤 鍍忎娇鐢? `M3508` 涓€鏍凤紝鍒涘缓 `MyMotor` 瀵硅薄锛屽苟灏嗗叾娉ㄥ唽鍒? `fdCANbus` 鍗冲彲銆傝皟搴﹀櫒浼氳嚜鍔ㄥ?鐞嗗悗缁?殑涓€鍒囥€?


