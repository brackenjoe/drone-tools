-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 Joe Fernandez
--[[
Caddx Gazer day/night: make the RC9 switch position absolute.

Built on a TBS Lucid H7 with the Gazer day/night pad wired to M9; runs
unchanged on ArduCopter 4.6.3 and 4.7.1. On other wiring, set RC_IN,
RELAY_IDX and RAIL_RELAY in the tuning block below.

Why this script exists
----------------------
The Gazer's day/night input is a momentary trigger, not a state input. Per the
Gazer quick start guide it is driven from a Betaflight PINIO pad and "triggers
the day/night mode switch each time": one pulse flips the camera to whatever
mode it is not in. There is no "set day" or "set night" command, so no amount
of switch remapping on the drone or the radio can make the camera follow a
level. Binding RC9 straight to Relay1 (RC9_OPTION = 28) gives the camera a
level, which is why it only changed on a full down-and-back-up cycle.

This script closes the loop: it keeps a model of the camera's current mode,
compares it to RC9, and pulses the relay exactly once when they disagree.

Keeping the model honest
------------------------
The camera always powers up in DAY (measured: set it to night, power cycle, and
it comes back in day every time). It never reports its state, so the model can
only stay correct if we know when the camera power-cycled.

On this airframe the Gazer and the VTX share a 9 V rail on RELAY4 (GPIO 83,
"VPWR"), toggled from RC6. That rail is usually OFF when the FC boots and gets
switched on later, so "reset the model at FC boot" is not good enough: the
camera has not even powered up yet. Measured on this airframe: video appears
8.69 s after the 9 V rail is switched on, so the guard has to clear that.

So the script tracks the rail instead. While the rail is off the camera is
unpowered, the model is pinned to DAY (what it will boot into), and no pulses
are sent. When the rail comes up the script waits CAM_BOOT_MS for the camera to
initialise and then brings it to whatever RC9 is asking for. That makes the
order of operations irrelevant: set the switch before or after powering video,
cycle the rail mid-flight, it converges either way.

Hardware path (verified on this airframe)
-----------------------------------------
  M9 pad = PWM output 9 = GPIO 58   (TBS_LUCID_H7 hwdef.dat line 153)
  SERVO9_FUNCTION = -1  (GPIO)
  RELAY1_PIN      = 58   day/night trigger, relay on = pin high = "pressed"
  RELAY4_PIN      = 83   PC13 VPWR, the 9 V camera + VTX rail, switched by RC6

Required parameter changes
--------------------------
  SCR_ENABLE      1      then reboot so SCR_USER1 appears
  RC9_OPTION      0      was 28 (Relay1 On/Off); the script owns the relay now
  RELAY1_DEFAULT  0      was 1; keeps M9 low at boot instead of high

Install to APM/scripts/gazer_daynight.lua on the microSD, then reboot.
--]]

-- ---------------------------------------------------------------- tuning ---

local RC_IN        = 9      -- RC channel carrying the two-position switch
local RELAY_IDX    = 0      -- relay instance 0 = RELAY1 = GPIO 58 = M9
local PRESS_MS     = 500    -- how long to hold the "button" down
local COOLDOWN_MS  = 1000   -- minimum gap between presses
local DEBOUNCE_MS  = 200    -- switch must be stable this long before acting
local STARTUP_MS   = 3000   -- minimum settle time after FC boot
local RC_MID       = 1500   -- threshold between the two switch positions
local LOOP_MS      = 50

-- The 9 V rail that powers the camera, as a relay instance (0 based, so
-- RELAY4 = 3). Set to -1 to disable rail tracking entirely, in which case the
-- script falls back to BOOT_ASSUME at FC boot only.
local RAIL_RELAY   = 3      -- RELAY4 = GPIO 83 = PC13 VPWR, toggled by RC6
local CAM_BOOT_MS  = 10000  -- settle time after the rail comes up.
                            -- Measured 2026-09-04: video appears 8.69 s after
                            -- the 9 V toggle, so this leaves ~1.3 s margin.

-- What mode the camera is in when it powers up. Only consulted when rail
-- tracking is disabled or the rail is already live at FC boot.
--   -1  camera remembers its last mode across power cycles (use SCR_USER1)
--    0  camera always powers up in DAY
--    1  camera always powers up in NIGHT
-- Measured on this airframe 2026-09-04: the Gazer does NOT remember.
local BOOT_ASSUME  = 0

local STATE_PARAM  = "SCR_USER1"   -- records current mode: 0 = DAY, 1 = NIGHT

-- ------------------------------------------------------------------ state ---

local DAY, NIGHT = 0, 1

local cam_mode                     -- what we believe the camera is doing
local phase = "idle"               -- idle | pressing | cooldown
local phase_until = 0
local cand, cand_since = nil, 0
local rail_up_at = nil             -- when the camera rail last came on

local function mode_name(m)
  if m == NIGHT then return "NIGHT" else return "DAY" end
end

if BOOT_ASSUME == DAY or BOOT_ASSUME == NIGHT then
  cam_mode = BOOT_ASSUME
else
  local saved = param:get(STATE_PARAM)
  if saved ~= nil and saved >= 0.5 then
    cam_mode = NIGHT
  else
    cam_mode = DAY
  end
end

gcs:send_text(6, string.format("Gazer v4: assuming %s, camera guard %.1fs",
                               mode_name(cam_mode), CAM_BOOT_MS / 1000))

-- ------------------------------------------------------------------- loop ---

local function rail_is_on()
  if RAIL_RELAY < 0 then
    return true                    -- rail tracking disabled: assume powered
  end
  return relay:get(RAIL_RELAY) == 1
end

function update()
  local now = millis():toint()

  -- finish an in-flight press before anything else
  if phase == "pressing" then
    if now >= phase_until then
      relay:off(RELAY_IDX)
      cam_mode = 1 - cam_mode
      param:set_and_save(STATE_PARAM, cam_mode)
      gcs:send_text(6, "Gazer: now " .. mode_name(cam_mode))
      phase = "cooldown"
      phase_until = now + COOLDOWN_MS
    end
    return update, LOOP_MS
  end

  -- not pressing: hold the line released
  relay:off(RELAY_IDX)

  -- camera unpowered: it will come up in DAY, so pin the model there and
  -- never spend a pulse on a device that cannot see it
  if not rail_is_on() then
    if rail_up_at ~= nil then
      gcs:send_text(6, "Gazer: camera rail off, model reset to DAY")
    end
    rail_up_at = nil
    cam_mode = DAY
    cand = nil
    return update, LOOP_MS
  end

  if rail_up_at == nil then
    rail_up_at = now
    cam_mode = DAY
    gcs:send_text(6, "Gazer: camera rail on, assuming DAY")
  end

  if phase == "cooldown" then
    if now < phase_until then return update, LOOP_MS end
    phase = "idle"
  end

  if now < STARTUP_MS then return update, LOOP_MS end
  if now - rail_up_at < CAM_BOOT_MS then return update, LOOP_MS end
  if not rc:has_valid_input() then return update, LOOP_MS end

  local pwm = rc:get_pwm(RC_IN)
  if pwm == nil then return update, LOOP_MS end

  local want = DAY
  if pwm > RC_MID then want = NIGHT end

  if want ~= cand then
    cand = want
    cand_since = now
    return update, LOOP_MS
  end
  if now - cand_since < DEBOUNCE_MS then return update, LOOP_MS end

  if want ~= cam_mode then
    relay:on(RELAY_IDX)
    phase = "pressing"
    phase_until = now + PRESS_MS
    gcs:send_text(6, "Gazer: switching to " .. mode_name(want))
  end

  return update, LOOP_MS
end

return update, LOOP_MS
