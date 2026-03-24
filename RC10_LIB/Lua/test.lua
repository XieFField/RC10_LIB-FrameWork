local function run(event)
    lcd.clear()
    
    -- 读取数据
    local spear_val = getValue("Curr") or 0    -- Battery Current
    local kfs1_val = getValue("Capa") or 0     -- Battery Capacity  
    local kfs2_val = getValue("Bat%") or 0     -- Battery Remaining (注意你写的是Bat%，通常是Rema)
    local gspd = getValue("GSpd") or 0         -- GPS Speed (X)
    local hdg = getValue("Hdg") or 0           -- GPS Course (Yaw)
    local galt = getValue("Alt") or 0         -- GPS Alt (Y)
    
    -- 转换计算
    local x = gspd / 10.0
    local y = (galt + 1000) / 100.0
    local yaw = hdg
    
      -- ========== Robot 位姿（X 和 Y 分开显示，间隔加大）==========
    -- X 坐标（左侧）
    lcd.drawText(0, 13, "X:" .. string.format("%.2f", x), SMLSIZE)
    -- Y 坐标（右侧，X=70，间隔加大）
    lcd.drawText(70, 13, "Y:" .. string.format("%.2f", y), SMLSIZE)
    
    -- Yaw 角度（单独一行）
    lcd.drawText(0, 26, "Yaw:" .. string.format("%.1f", yaw) .. "°", SMLSIZE)

  -- KFS1（左侧，显示为整数）
    lcd.drawText(0, 39, "KFS1:" .. string.format("%d", kfs1_val), SMLSIZE)
    -- KFS2（右侧，X=70，间隔加大，显示为整数）
    lcd.drawText(70, 39, "KFS2:" .. string.format("%d", kfs2_val), SMLSIZE)

    -- Spear 状态（底部，显示为整数）
    lcd.drawText(0, 52, "Spear:" .. string.format("%d", spear_val), SMLSIZE)
    
    return 0
end

return { run = run }