-- SPDX-License-Identifier: GPL-3.0-or-later
-- Copyright (C) 2026 Joe Fernandez
-- Stubbed ArduPilot environment to exercise gazer_daynight.lua
-- Run: lua5.3 gazer_daynight_test.lua [path-to-script]
--
-- lua5.3, not 5.4: ArduPilot vendors Lua 5.3 (5.3.5 in 4.6, 5.3.6 in 4.7), and 5.4 accepts syntax 5.3
-- rejects, so passing here under 5.4 would not mean passing on the aircraft.
--
-- These globals are read-only on a real vehicle, provided by the firmware.
-- This file replaces them on purpose, which is the whole point of a test
-- double, so tell luacheck that rather than carrying five warnings forever.
-- luacheck: globals millis gcs param rc relay

-- Default to the script sitting next to this file, resolved from arg[0], so
-- the test runs from any working directory.
local HERE = ((arg and arg[0]) or ""):match("(.*/)") or "./"
local SCRIPT = (arg and arg[1]) or (HERE .. "gazer_daynight.lua")

local RAIL = 3   -- relay instance 3 = RELAY4 = VPWR, the 9V camera rail

local T, relay_state, rail_state, pulses, params, rc_pwm, rc_ok, log

local function reset(saved_state, switch_pwm, rail)
  T = 0
  relay_state = 0
  rail_state = rail and 1 or 0
  pulses = 0
  params = { SCR_USER1 = saved_state }
  rc_pwm = switch_pwm
  rc_ok = true
  log = {}
end

millis = function()
  local v = T
  return { toint = function() return v end }
end

gcs = { send_text = function(_, _, t) log[#log+1] = t end }

param = {
  get = function(_, n) return params[n] end,
  set_and_save = function(_, n, v) params[n] = v; return true end,
}

rc = {
  has_valid_input = function() return rc_ok end,
  get_pwm = function(_, ch) if ch ~= 9 then return nil end return rc_pwm end,
}

relay = {
  on = function(_, i)
    assert(i == 0, "only the day/night relay should ever be driven, got " .. tostring(i))
    if relay_state == 0 then pulses = pulses + 1 end
    relay_state = 1
  end,
  off = function(_, i)
    assert(i == 0, "only the day/night relay should ever be driven, got " .. tostring(i))
    relay_state = 0
  end,
  get = function(_, i)
    if i == RAIL then return rail_state end
    return relay_state
  end,
}

local cb, delay
local function run_for(ms)
  local stop = T + ms
  while T < stop do
    cb, delay = cb()
    T = T + delay
  end
end

local function boot(saved_state, switch_pwm, rail)
  reset(saved_state, switch_pwm, rail)
  cb, delay = dofile(SCRIPT)
  T = T + delay
end

local function variant(field, v)
  local f = assert(io.open(SCRIPT, "r"))
  local src = f:read("a"); f:close()
  src = src:gsub("local " .. field .. "%s*=%s*%-?%d+", "local " .. field .. " = " .. v, 1)
  local path = os.tmpname() .. ".lua"
  local o = assert(io.open(path, "w")); o:write(src); o:close()
  return path
end

local fails = 0
local function check(name, got, want)
  local ok = got == want
  if not ok then fails = fails + 1 end
  print(string.format("%-60s %s  (got %s, want %s)",
    name, ok and "PASS" or "FAIL", tostring(got), tostring(want)))
end

-- ===========================================================================
print("=== normal workflow: FC boots with the 9V rail OFF ===")
-- ===========================================================================

boot(1, 988, false)            -- stale SCR_USER1=NIGHT, switch at day, rail off
run_for(30000)
check("rail off -> no pulses at all", pulses, 0)
check("  ... day/night line held low", relay_state, 0)

rail_state = 1                 -- flip the 9V rail on
run_for(30000)
check("rail on, switch DAY -> still no pulse", pulses, 0)

rc_pwm = 2012
run_for(5000)
check("then switch -> NIGHT gives one pulse", pulses, 1)
check("  ... SCR_USER1 = 1", params.SCR_USER1, 1)

-- ===========================================================================
print()
print("=== the case that FC-boot-only logic gets wrong ===")
-- ===========================================================================

-- switch set to NIGHT before video power is turned on
boot(0, 2012, false)
run_for(30000)
check("switch at NIGHT while rail off -> no wasted pulse", pulses, 0)
rail_state = 1
run_for(8500)
check("  ... silent at 8.5s, video not up yet (measured 8.69s)", pulses, 0)
run_for(3000)
check("  ... fires just after the 10s guard clears", pulses, 1)
check("  ... camera ends in NIGHT as the switch asks", params.SCR_USER1, 1)

-- ===========================================================================
print()
print("=== rail cycled mid-session while the switch says NIGHT ===")
-- ===========================================================================

boot(0, 988, true)
run_for(30000)
rc_pwm = 2012; run_for(10000)
check("at NIGHT before the power cycle", pulses, 1)
rail_state = 0                 -- video power off; camera forgets, reverts to day
run_for(5000)
check("  ... rail off adds no pulse", pulses, 1)
rail_state = 1                 -- back on; camera is now physically in DAY
run_for(30000)
check("  ... rail back on -> one corrective pulse", pulses, 2)
check("  ... camera back to NIGHT", params.SCR_USER1, 1)

-- ===========================================================================
print()
print("=== ordinary toggling with the rail up ===")
-- ===========================================================================

boot(0, 988, true)
run_for(30000)
check("settled at DAY, no pulse", pulses, 0)
rc_pwm = 2012; run_for(5000)
check("-> NIGHT: one pulse", pulses, 1)
run_for(30000)
check("holding at NIGHT: nothing further", pulses, 1)
rc_pwm = 988; run_for(5000)
check("-> DAY: one more pulse", pulses, 2)
run_for(30000)
check("holding at DAY: nothing further", pulses, 2)
check("  ... line left low", relay_state, 0)

-- debounce and failsafe
boot(0, 988, true)
run_for(30000)
for _ = 1, 3 do
  rc_pwm = 2012; run_for(100)
  rc_pwm = 988;  run_for(100)
end
check("sub-debounce chatter emits no pulse", pulses, 0)

boot(0, 2012, true)
rc_ok = false
run_for(30000)
check("no valid RC input -> no pulse", pulses, 0)
rc_ok = true
run_for(10000)
check("  ... resumes once RC returns", pulses, 1)

-- ===========================================================================
print()
print("=== fallback: RAIL_RELAY = -1 (rail tracking disabled) ===")
-- ===========================================================================
local norail = variant("RAIL_RELAY", -1)
local boot_norail = function(saved, pwm, rail)
  reset(saved, pwm, rail)
  cb, delay = dofile(norail)
  T = T + delay
end

boot_norail(1, 988, false)
run_for(30000)
check("ignores the rail, switch DAY -> no pulse", pulses, 0)

boot_norail(1, 2012, false)
run_for(30000)
check("ignores the rail, switch NIGHT -> one pulse", pulses, 1)
os.remove(norail)

print()
if fails == 0 then print("ALL TESTS PASSED") else print(fails .. " TEST(S) FAILED") end
os.exit(fails == 0 and 0 or 1)
