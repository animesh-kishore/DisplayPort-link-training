# DisplayPort-link-training
State machine driver which handles DisplayPort full and fast link training.

For diagrammatic representation of control flow check lt_state_machine_diagram.jpg. 

The state machine driver is compliant to DisplayPort v1.2. It's a minor effort to extend the driver to DisplayPort v1.3 and v1.4.

Compliance testing is done on Unigraf DPR-120 analyzer. In fact, VESA has already approved our DisplayPort driver bearing this link training state machine, compliant for one of the chipsets.

All arch specific programming required for link training goes as ops in struct dp_lt_ops. Not all ops are mandatory. Non-mandatory ops are mainly to impose arch specific restrictions on state machine. e.g Older DisplayPort v1.1 would not support 5.4Gbps link bandwidth i.e. HBR2. Archs targeting only eDP might not support 4 lanes. Please read comments in dp_lt.h for detailed description for each ops.

You must initialize state machine before usage, API dp_lt_init(). Init function has no arch specific or panel specific calls. This means that initialization can be done even while HW is not ready.

All link training requests are initiated via dp_lt_set_pending_evt(). Default state transitions should be good for most archs and usecases. For those who might want to customize state transitions can use APIs dp_lt_invalidate() and dp_lt_force_trigger(). Read dp_lt.c API description for detailed usage.

Suspend/Resume policy for display can get tricky across archs. e.g mobile platforms typically do not process hotplugs during suspend. On other hand set top boxes might have their display interfaces always ready to process hotplugs even during suspend. For former case you might want state machine to transition to stop state during suspend even if monitor is still connected. APIs dp_lt_force_disable() and dp_lt_wait_for_completion() can be used to handle such cases. Detailed API description for the same in dp_lt.c
