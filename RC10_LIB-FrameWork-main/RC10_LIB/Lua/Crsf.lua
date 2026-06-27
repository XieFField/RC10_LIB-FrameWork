-- 鐘舵€佽?蹇嗗彉閲忥紙鐢ㄤ簬杈规部妫€娴嬶級
Last_control_mode = 0
Last_spear = 0

local function my_init()
    -- init 鍦ㄦā鍨嬪姞杞芥椂璋冪敤涓€娆?
end

local function my_background()
    -- background 鍛ㄦ湡鎬ц皟鐢?
end

local function my_run(event)
    lcd.clear()

    -- 鑾峰彇鍘熷?鏁版嵁锛堝畾鐐规暟锛屾斁澶?100鍊嶏級
    local x_raw = getValue("0C10") or 0       -- Robot X
    local y_raw = getValue("0C11") or 0       -- Robot Y  
    local yaw_raw = getValue("0C12") or 0     -- Robot Yaw
    local kfs_x_raw = getValue("0C20") or 0   -- KFS X
    local kfs_y_raw = getValue("0C21") or 0   -- KFS Y
    local spear_raw = getValue("0C30") or 0   -- Spear
    local mode_raw = getValue("0C40") or 0    -- Control Mode
 
    -- 鏈夌?鍙疯浆鎹?紙int16 鑼冨洿锛?-32768~32767锛?
    if x_raw > 32767 then x_raw = x_raw - 65536 end
    if y_raw > 32767 then y_raw = y_raw - 65536 end
    if yaw_raw > 32767 then yaw_raw = yaw_raw - 65536 end
    if kfs_x_raw > 32767 then kfs_x_raw = kfs_x_raw - 65536 end
    if kfs_y_raw > 32767 then kfs_y_raw = kfs_y_raw - 65536 end
    if spear_raw > 32767 then spear_raw = spear_raw - 65536 end
    if mode_raw > 32767 then mode_raw = mode_raw - 65536 end

    -- 杩樺師涓? float锛堥櫎浠?100锛屼繚鐣?2浣嶅皬鏁帮級
    local x = x_raw / 100.0
    local y = y_raw / 100.0
    local yaw = yaw_raw / 100.0
    local kfs_x = kfs_x_raw / 100.0
    local kfs_y = kfs_y_raw / 100.0
    local spear = spear_raw / 100.0
    local mode = mode_raw / 100.0

    -- 椤堕儴鏄剧ず鎺у埗妯″紡锛圷=0锛?
    local mode_int = math.floor(mode + 0.5)
    if mode_int == 1 then
        lcd.drawText(0, 0, "Mode1", MIDSIZE)
    elseif mode_int == 2 then
        lcd.drawText(0, 0, "Mode2", MIDSIZE)
    elseif mode_int == 3 then
        lcd.drawText(0, 0, "Mode3", MIDSIZE)
    elseif mode_int == 4 then
        lcd.drawText(0, 0, "Mode4", MIDSIZE + INVERS + BLINK)
    elseif mode_int == 5 then
        lcd.drawText(0, 0, "Mode5", MIDSIZE)
    elseif mode_int == 6 then
        lcd.drawText(0, 0, "Mode6", MIDSIZE)
    else
        lcd.drawText(10, 0, "GDUT Robocon2026", MIDSIZE)
    end


    -- ========== Robot 浣嶅Э锛圶 鍜? Y 鍒嗗紑鏄剧ず锛岄棿闅斿姞澶э級==========
    -- X 鍧愭爣锛堝乏渚э級
    lcd.drawText(0, 13, "X:" .. string.format("%.2f", x), SMLSIZE)
    -- Y 鍧愭爣锛堝彸渚э紝X=70锛岄棿闅斿姞澶э級
    lcd.drawText(70, 13, "Y:" .. string.format("%.2f", y), SMLSIZE)
    
    -- Yaw 瑙掑害锛堝崟鐙?竴琛岋級
    lcd.drawText(0, 26, "Yaw:" .. string.format("%.2f", yaw), SMLSIZE)

    -- ========== KFS 鏁版嵁锛圶 鍜? Y 鍒嗗紑鏄剧ず锛岄棿闅斿姞澶э級==========
    -- KFS_X锛堝乏渚э級
    lcd.drawText(0, 39, "KFS_X:" .. string.format("%.2f", kfs_x), SMLSIZE)
    -- KFS_Y锛堝彸渚э紝X=70锛岄棿闅斿姞澶э級
    lcd.drawText(70, 39, "KFS_Y:" .. string.format("%.2f", kfs_y), SMLSIZE)

    -- Spear 鐘舵€侊紙搴曢儴锛?
    lcd.drawText(0, 52, "Spear:" .. string.format("%.2f", spear), SMLSIZE)

    -- 鏇存柊璁板繂鍙橀噺锛堢敤浜庤竟娌挎?娴嬶級
    Last_control_mode = mode_int
    Last_spear = spear_raw

    return 0
end

return { run = my_run, background = my_background, init = my_init }