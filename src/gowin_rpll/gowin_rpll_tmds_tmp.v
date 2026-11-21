//Copyright (C)2014-2026 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.12.04-1
//IP Version: 1.0
//Part Number: GW1NR-LV9QN88PC6/I5
//Device: GW1NR-9
//Created Time: Sat Nov 21 20:04:32 2026

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

    Gowin_rPLL_TMDS your_instance_name(
        .clkout(clkout), //output clkout
        .lock(lock), //output lock
        .reset_p(reset_p), //input reset_p
        .clkin(clkin) //input clkin
    );

//--------Copy end-------------------
