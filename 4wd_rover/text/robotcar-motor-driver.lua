-- robotcar motor driver

local chNum_main1 = 94  -- k_scripting1
local chNum_main2 = 95  -- k_scripting2
local chNum_main3 = 96  -- k_scripting3
local chNum_main4 = 97  -- k_scripting4
local chNum_main5 = 98  -- k_scripting5
local chNum_main6 = 99  -- k_scripting6

local chNum_aux1 = 102  -- k_scripting9
local chNum_aux2 = 103  -- k_scripting10
local chNum_aux3 = 104  -- k_scripting11
local chNum_aux4 = 105  -- k_scripting12
local chNum_aux5 = 106  -- k_scripting13
local chNum_aux6 = 107  -- k_scripting14

local pwm_hi = 2000
local pwm_low = 0
local pwm_mid = 1500
local pwm_neutral_hi = 1550
local pwm_neutral_low = 1450

-- 正回転
local pwm1f_scale = 0.50
local pwm2f_scale = 0.85
local pwm3f_scale = 1.0
local pwm4f_scale = 0.50

-- 逆回転
local pwm1b_scale = 0.55
local pwm2b_scale = 0.85
local pwm3b_scale = 1.0
local pwm4b_scale = 0.55

-- 調整倍率（格納用）
local pwm1_scale = 0
local pwm2_scale = 0
local pwm3_scale = 0
local pwm4_scale = 0

function pwm_control(scale, pwm)
  -- pwm値 調節
  local mod_pwm = pwm_mid

  if pwm > pwm_mid then
    mod_pwm = ((pwm - pwm_mid) * scale) + pwm_mid
  else
    mod_pwm = pwm_mid - ((pwm_mid - pwm) * scale)
  end
  
  -- gcs:send_text(0, "pwm_control:" .. tostring(math.floor(mod_pwm)))
  return math.floor(mod_pwm)
end

function rc_neutral(pwm)
  -- neutral 判定
  local ret_flg = false

  if pwm_neutral_low < pwm and pwm < pwm_neutral_hi then
    ret_flg = true
  end
  
  return ret_flg
end

function switch_move(pwm_aile, pwm_elev)
  -- 移動
  if rc_neutral(pwm_elev) == false and rc_neutral(pwm_aile) then
    move_upperLower(pwm_elev)
  elseif rc_neutral(pwm_aile) == false and rc_neutral(pwm_elev) then
    move_rightLeft(pwm_aile)
  else
    move_stop()
  end
end

-- Upper Left(左上)		Upper Middle(上中央)	Upper Right(右上)
-- Middle Left(左中央)	Middle(中央)			Middle Right(右中央)
-- Lower Left(左下)		Lower Middle(下中央)	Lower Right(右下)

function move_upperLower(pwm_elev)
  if pwm_elev >= pwm_mid then
    pwm1_scale = pwm1f_scale
    pwm2_scale = pwm2f_scale
    pwm3_scale = pwm3f_scale
    pwm4_scale = pwm4f_scale
  else
    pwm1_scale = pwm1b_scale
    pwm2_scale = pwm2b_scale
    pwm3_scale = pwm3b_scale
    pwm4_scale = pwm4b_scale
  end
  -- 前進/後進
  gcs:send_text(0, "前進/後進") -- .. tostring(pwm_control(pwm1_scale, pwm_elev)) .. tostring(pwm_control(pwm3_scale, pwm_elev)))
  SRV_Channels:set_output_pwm(chNum_main1, pwm_control(pwm1_scale, 3000-pwm_elev))
  SRV_Channels:set_output_pwm(chNum_main2, pwm_control(pwm2_scale, pwm_elev))
  SRV_Channels:set_output_pwm(chNum_main3, pwm_control(pwm3_scale, pwm_elev))
  SRV_Channels:set_output_pwm(chNum_main4, pwm_control(pwm4_scale, 3000-pwm_elev))
end

function move_rightLeft(pwm_aile)
  if pwm_aile >= pwm_mid then
    pwm1_scale = pwm1f_scale
    pwm2_scale = pwm2f_scale
    pwm3_scale = pwm3f_scale
    pwm4_scale = pwm4f_scale
  else
    pwm1_scale = pwm1b_scale
    pwm2_scale = pwm2b_scale
    pwm3_scale = pwm3b_scale
    pwm4_scale = pwm4b_scale
  end
  -- 右進/左進
  gcs:send_text(0, "右進/左進")
  SRV_Channels:set_output_pwm(chNum_main1, pwm_control(pwm1_scale, pwm_aile))
  SRV_Channels:set_output_pwm(chNum_main2, pwm_control(pwm2_scale, pwm_aile))
  SRV_Channels:set_output_pwm(chNum_main3, pwm_control(pwm3_scale, 3000-pwm_aile))
  SRV_Channels:set_output_pwm(chNum_main4, pwm_control(pwm4_scale, 3000-pwm_aile))
end

function move_stop()
  -- 停止
  -- gcs:send_text(0, "停止")
  SRV_Channels:set_output_pwm(chNum_main1, pwm_control(pwm1_scale, pwm_mid))
  SRV_Channels:set_output_pwm(chNum_main2, pwm_control(pwm2_scale, pwm_mid))
  SRV_Channels:set_output_pwm(chNum_main3, pwm_control(pwm3_scale, pwm_mid))
  SRV_Channels:set_output_pwm(chNum_main4, pwm_control(pwm4_scale, pwm_mid))
end

function switch_turn(pwm_rudd)
  if pwm_rudd >= pwm_mid then
    pwm1_scale = pwm1f_scale
    pwm2_scale = pwm2f_scale
    pwm3_scale = pwm3f_scale
    pwm4_scale = pwm4f_scale
  else
    pwm1_scale = pwm1b_scale
    pwm2_scale = pwm2b_scale
    pwm3_scale = pwm3b_scale
    pwm4_scale = pwm4b_scale
  end
  -- 回転
  gcs:send_text(0, "回転")
  SRV_Channels:set_output_pwm(chNum_main1, pwm_control(pwm1_scale, pwm_rudd))
  SRV_Channels:set_output_pwm(chNum_main2, pwm_control(pwm2_scale, pwm_rudd))
  SRV_Channels:set_output_pwm(chNum_main3, pwm_control(pwm3_scale, pwm_rudd))
  SRV_Channels:set_output_pwm(chNum_main4, pwm_control(pwm4_scale, pwm_rudd))
end

function update()
  local pwm_aile = rc:get_pwm(1)  -- Aileron
  local pwm_elev = rc:get_pwm(2)  -- Elevator
  local pwm_thro = rc:get_pwm(3)  -- Throttle
  local pwm_rudd = rc:get_pwm(4)  -- Rudder
  -- gcs:send_text(0, "RCIN 1(Aileron):" .. tostring(pwm_aile) .. " 2(Elevator):" .. tostring(pwm_elev).. " 3(Throttle):" .. tostring(pwm_thro).. " 4(Rudder):" .. tostring(pwm_rudd))
  
  if rc_neutral(pwm_rudd) == false then
    switch_turn(pwm_rudd)
  else
    switch_move(pwm_aile, pwm_elev)
  end
  
  return update, 10 -- run at 100hz
end

-- gcs:send_text(6, "robotcar-motor-driver.lua is running")
return update()
