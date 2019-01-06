
-- registration function
function bind_events(t, verbose)
   for key,value in pairs(t) do
      xcsoar.input_event.new(key,
                             function(e)
                                if (verbose) then
                                   print(key); -- logging of events
                                end
                                value(e);
                             end
      )
   end
end

-- Glide computer event handlers ----------------------------------------------

function beep()
   xcsoar.fire_legacy_event("Beep","1");
end

function on_task_transition(mode)
   beep();
   xcsoar.fire_legacy_event("TaskTransition",mode);
end

function on_final_glide_event(mode)
   xcsoar.fire_legacy_event("StatusMessage",mode);
end

-- Glide computer event table -------------------------------------------------

local gce_events = {
  ["gce_takeoff"] = function(e)
     xcsoar.fire_legacy_event("AutoLogger","start");
     xcsoar.fire_legacy_event("AddWaypoint","takeoff");
     xcsoar.fire_legacy_event("StatusMessage","Takeoff");
  end,
  ["gce_landing"] = function(e)
     xcsoar.fire_legacy_event("StatusMessage","Landing");
     xcsoar.fire_legacy_event("AutoLogger","stop");
  end,
  ["gce_flightmode_finalglide_above"] = function(e)
     on_final_glide_event("Above final glide");
  end,
  ["gce_flightmode_finalglide_below"] = function(e)
     on_final_glide_event("Below final glide");
  end,
  ["gce_flightmode_finalglide_terrain"] = function(e)
     on_final_glide_event("Final glide through terrain");
  end,
  ["gce_landable_unreachable"] = beep,
  ["gce_task_start"] = function(e)
     on_task_transition("start");
  end,
  ["gce_task_finish"] = function(e)
     on_task_transition("finish");
  end,
  ["gce_task_nextwaypoint"] = function(e)
     on_task_transition("next");
  end,
};

-- Bind events ---------------------------------------------------------------

local verbose = 1;
bind_events(gce_events, verbose);

