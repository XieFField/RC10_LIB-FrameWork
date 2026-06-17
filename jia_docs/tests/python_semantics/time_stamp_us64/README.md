# `BSP_RtosTimeStampUs64` 鍥炲綊娴嬭瘯璇存槑

## 鐩爣
- 楠岃瘉鍐呴儴鎹㈢畻璇箟锛堥浂闄や繚鎶ゃ€佹暣闄よ涓恒€佸瓙 tick 鎴柇锛変繚鎸佺ǔ瀹氥€?
- 楠岃瘉瀵瑰鎺ュ彛宸叉敹鍙ｄ负闆跺弬鏁版椂闂存埑鎺ュ彛锛歚getTimeUs()`銆?
- 楠岃瘉鍛藉悕绌洪棿宸茬粺涓€涓?`namespace jia`銆?

## 杩愯鏂瑰紡
```powershell
python jia_docs/tests/ai2_tests/time_stamp_us64/time_stamp_us64_regression.py
```

## 瑕嗙洊璇存槑
- `tick_rate_hz == 0` 杩斿洖 `0`銆?
- 甯歌 `ticks -> us` 鎹㈢畻缁撴灉姝ｇ‘銆?
- `sub_tick_us > tick_period_us` 鏃舵墽琛屾埅鏂€?
- 闈欐€佺害鏉燂細
  - BSP 澶?婧愬瓨鍦?`class RtosTimeStampUs64`
  - 澶存枃浠跺寘鍚?`getTimeUs`锛屼笉鍐嶆毚闇?`TicksToUs64/ComposeTimeUs64`
  - 婧愮爜鍛藉悕绌洪棿涓?`namespace jia`
