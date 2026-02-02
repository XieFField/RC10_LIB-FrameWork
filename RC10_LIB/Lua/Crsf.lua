-- 状态记忆变量（用于边沿检测）
Last_control_mode = 0
Last_spear = 0

local function my_init()
    -- init 在模型加载时调用一次
end

local function my_background()
    -- background 周期性调用
end

local function my_run(event)
    lcd.clear()

    -- 获取原始数据（定点数，放大100倍）
    local x_raw = getValue("0C10") or 0       -- Robot X
    local y_raw = getValue("0C11") or 0       -- Robot Y  
    local yaw_raw = getValue("0C12") or 0     -- Robot Yaw
    local kfs_x_raw = getValue("0C20") or 0   -- KFS X
    local kfs_y_raw = getValue("0C21") or 0   -- KFS Y
    local spear_raw = getValue("0C30") or 0   -- Spear
    local mode_raw = getValue("0C40") or 0    -- Control Mode

    -- 有符号转换（int16 范围：-32768~32767）
    if x_raw > 32767 then x_raw = x_raw - 65536 end
    if y_raw > 32767 then y_raw = y_raw - 65536 end
    if yaw_raw > 32767 then yaw_raw = yaw_raw - 65536 end
    if kfs_x_raw > 32767 then kfs_x_raw = kfs_x_raw - 65536 end
    if kfs_y_raw > 32767 then kfs_y_raw = kfs_y_raw - 65536 end
    if spear_raw > 32767 then spear_raw = spear_raw - 65536 end
    if mode_raw > 32767 then mode_raw = mode_raw - 65536 end

    -- 还原为 float（除以100，保留2位小数）
    local x = x_raw / 100.0
    local y = y_raw / 100.0
    local yaw = yaw_raw / 100.0
    local kfs_x = kfs_x_raw / 100.0
    local kfs_y = kfs_y_raw / 100.0
    local spear = spear_raw / 100.0
    local mode = mode_raw / 100.0

    -- 顶部显示控制模式（Y=0）
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

    -- ========== Robot 位姿（X 和 Y 分开显示，间隔加大）==========
    -- X 坐标（左侧）
    lcd.drawText(0, 13, "X:" .. string.format("%.2f", x), SMLSIZE)
    -- Y 坐标（右侧，X=70，间隔加大）
    lcd.drawText(70, 13, "Y:" .. string.format("%.2f", y), SMLSIZE)
    
    -- Yaw 角度（单独一行）
    lcd.drawText(0, 26, "Yaw:" .. string.format("%.2f", yaw), SMLSIZE)

    -- ========== KFS 数据（X 和 Y 分开显示，间隔加大）==========
    -- KFS_X（左侧）
    lcd.drawText(0, 39, "KFS_X:" .. string.format("%.2f", kfs_x), SMLSIZE)
    -- KFS_Y（右侧，X=70，间隔加大）
    lcd.drawText(70, 39, "KFS_Y:" .. string.format("%.2f", kfs_y), SMLSIZE)

    -- Spear 状态（底部）
    lcd.drawText(0, 52, "Spear:" .. string.format("%.2f", spear), SMLSIZE)

    -- 更新记忆变量（用于边沿检测）
    Last_control_mode = mode_int
    Last_spear = spear_raw

    return 0
end

return { run = my_run, background = my_background, init = my_init }