-- Gestures -------------------------------------------------------------------

xcsoar.input_event.new("gesture_U",
                                function(e)
                                   xcsoar.fire_legacy_event("Zoom","in");
                                   e:cancel();
                                end
)

xcsoar.input_event.new("gesture_D",
                                function(e)
                                   xcsoar.fire_legacy_event("Zoom","out");
                                end
)

xcsoar.input_event.new("gesture_UD",
                                function(e)
                                   xcsoar.fire_legacy_event("Zoom","auto on");
                                   xcsoar.fire_legacy_event("Zoom","auto show");
                                end
)

xcsoar.input_event.new("gesture_R",
                                function(e)
                                   xcsoar.fire_legacy_event("ScreenModes","previous");
                                end
)

xcsoar.input_event.new("gesture_L",
                                function(e)
                                   xcsoar.fire_legacy_event("ScreenModes","next");
                                end
)

xcsoar.input_event.new("gesture_DR",
                                function(e)
                                   xcsoar.fire_legacy_event("WaypointDetails","select");
                                end
)

xcsoar.input_event.new("gesture_DL",
                                function(e)
                                   xcsoar.fire_legacy_event("Setup","Alternates");
                                end
)

xcsoar.input_event.new("gesture_DU",
                                function(e)
                                   xcsoar.fire_legacy_event("Mode","Menu");
                                end
)

xcsoar.input_event.new("gesture_RD",
                                function(e)
                                   xcsoar.fire_legacy_event("Calculator");
                                end
)

xcsoar.input_event.new("gesture_URD",
                                function(e)
                                   xcsoar.fire_legacy_event("Analysis");
                                end
)

xcsoar.input_event.new("gesture_URDL",
                                function(e)
                                   xcsoar.fire_legacy_event("Pan", "on");
                                end
)

xcsoar.input_event.new("gesture_LDRDL",
                                function(e)
                                   xcsoar.fire_legacy_event("Status", "all");
                                end
)
