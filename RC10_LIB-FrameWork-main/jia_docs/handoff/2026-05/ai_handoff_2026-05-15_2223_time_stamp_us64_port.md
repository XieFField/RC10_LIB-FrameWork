# 鏃堕棿鎴虫寜鏃堕挓婧愭敹鏁涗氦鎺ユ枃妗ｏ紙RTOS+SysTick锛?

鐢熸垚鏃堕棿锛?026-05-15 22:23锛圓sia/Shanghai锛? 
浠撳簱璺緞锛歚D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`  
褰撳墠鍒嗘敮锛歚Jia6_temp`  
鏂囨。璺緞锛歚D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\handoff\ai_handoff_2026-05-15_2223_time_stamp_us64_port.md`

## 1. 鏈疆鐩爣涓庣粨鏋?

- 鐩爣锛氬皢 RTOS+SysTick 鐨?64 浣嶆椂闂存埑瀹炵幇鏀舵暃鍒?BSP锛屽苟鎻愬崌鎺ュ彛灏佽搴︺€? 
- 缁撴灉锛氬凡瀹屾垚鐩綍鏀舵暃銆佸懡鍚嶇┖闂寸粺涓€銆佹帴鍙ｆ敹鍙ｏ紝涓斾笉褰卞搷鍘熸湁 `BSP_TimeStamp` 涓婚摼璺€?

## 2. 褰撳墠鐪熷疄鐘舵€侊紙鍏抽敭缁撹锛?

1. TIM 鏃堕挓婧愭椂闂存埑浠嶇敱 `BSP_TimeStamp` 鎻愪緵锛屼笟鍔′富閾捐矾涓嶅彉銆? 
2. RTOS+SysTick 瀹炵幇浣嶄簬锛? 
   - `RC10_LIB/BSP_Driver/Inc/BSP_RtosTimeStampUs64.h`  
   - `RC10_LIB/BSP_Driver/Src/BSP_RtosTimeStampUs64.cpp`  
3. 鍛藉悕绌洪棿宸茬粺涓€涓?`namespace jia`锛堜笉鍐嶄娇鐢?`jia::time`锛夈€? 
4. 瀵瑰鎺ュ彛浠呬繚鐣欙細`RtosTimeStampUs64::getTimeUs()`銆? 
5. `ticksToUs64/composeTimeUs64` 宸蹭笅娌夊埌 `cpp` 鍖垮悕鍛藉悕绌洪棿锛屼粎鍐呴儴鍙銆?

## 3. 宸ョ▼鎺ョ嚎鐘舵€?

- `MDK-ARM/Frame_T.uvprojx` 宸插寘鍚細
  - `BSP_RtosTimeStampUs64.h`
  - `BSP_RtosTimeStampUs64.cpp`
- APP 灞傚吋瀹瑰ご `APP_TimeStampUs64.h` 宸茬Щ闄わ紝涓嶅啀淇濈暀 APP 鏃堕棿鎴冲叆鍙ｃ€?

## 4. 娴嬭瘯璧勪骇涓庨獙璇?

娴嬭瘯璧勪骇浣嶇疆锛? 
`jia_docs/tests/ai2_tests/time_stamp_us64/`

瑕嗙洊瑕佺偣锛? 
1. 闆堕櫎淇濇姢涓庢暣闄ゆ崲绠? 
2. 瀛?tick 鎴柇璇箟  
3. 闈欐€佹帴鍙ｇ害鏉燂紙`namespace jia` + `getTimeUs` 瀵瑰鍞竴鍏ュ彛锛?

寤鸿鎵ц锛? 
```powershell
python jia_docs/tests/ai2_tests/time_stamp_us64/time_stamp_us64_regression.py
```

## 5. 浣跨敤寤鸿锛堟寜鏃堕挓婧愶級

- 闇€瑕?TIM 杩炵画鏃跺熀锛氱户缁娇鐢?`TimeStamp::getInstance()`銆? 
- 闇€瑕?RTOS Tick + SysTick 缁勫悎鏃跺熀锛氫娇鐢?`jia::RtosTimeStampUs64::getTimeUs()`銆? 

## 6. 涓€鍙ヨ瘽缁撹

> 鏃堕棿鎴虫ā鍧楀凡鎸夆€滄椂閽熸簮鈥濆畬鎴愮粨鏋勬敹鏁涳紝RTOS+SysTick 渚ф帴鍙ｅ仛鍒伴浂鍙傛暟鍙栧€硷紝璋冪敤鏂规棤闇€鍏冲績鎹㈢畻缁嗚妭锛岀淮鎶よ竟鐣屾洿娓呮櫚銆?
