This folder contains the MB1 CFGs for the GPIO pin interrupt
routing from different line of GPIO controller.

In T19x, each GPO controller have the 8 interrupts in LIC instead
of 1 (legacy, parker or before). This gives opportunity to map the
GPIO pin to any of 8 interrupts. The design is done to reduce the
interrupt hunt time for pins from single ISRs.


MB1 CFG Format
==============
Prefix
	Interrupt mapping parameters for CFG start with prefix gpio-intmap.

Version
	Specify the major and minor number. Must be 1.0.
		gpio-intmap.major = 1;
		gpio-intmap.minor = 0;

Port number-data Format
	The data format in MB1 CFG for GPIO interrupt mapping is:
		gpio-intmap.<port>.<pin-id> = <interrupt-id>; # Comment

		Here:
			gpio-intmap:
				It is domain name of the MB1 CFG data.
				This is applicable for GPIO interrupt mapping.
			port:
				The port name like A, B, C..Z, AA, BB
				This can be in small or caps letter.
			pin-id:
				Pin ID in that port. It will be 0 to 7.
				E.g. GPIO PB0 means Port B, pin 0.
			interrupt-id:
				Interrupt route for that pin. It will be will
				be 0 to 7.

		Example:
			gpio-intmap.B.0 = 3;

				Means Pin GPIO PB0 interrupt routed to 3rd
				interrupt of that GPIO controller.

Creation of MB1 CFG file:
=======================
	This is manual edited file. Take file as
		"tegra194-mb1-bct-gpio-int-to-all-int0.cfg"
	as sample, copy it and start modififying per your requirements.

