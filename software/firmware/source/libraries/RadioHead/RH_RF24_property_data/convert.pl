#!/usr/bin/perl
#
# Convert a RH_RFM24 dump of a desired modulation made with printRegisters into an entry suitable for 
# inclusion in RH_RF24::ModemConfig MODEM_CONFIG_TABLE

use strict;

# List of the properties that are relevant to modulation schemes and speeds
my @wanted_properties = ( 
    0x2000,
    0x2003,
    0x2004,
    0x2005,
    0x2006,
    0x2007,
    0x2008,
    0x2009,
    0x200a,
    0x200b,
    0x200c,
    0x2018,
    0x201e,
    0x201f,
    0x2022,
    0x2023,
    0x2024,
    0x2025,
    0x2026,
    0x2027,
    0x2028,
    0x2029,
    0x202d,
    0x202e,
    0x202f,
    0x2030,
    0x2031,
    0x2035,
    0x2038,
    0x2039,
    0x203a,
    0x203b,
    0x203c,
    0x203d,
    0x203e,
    0x203f,
    0x2040,
    0x2043,
    0x2045,
    0x2046,
    0x2047,
    0x204e,
    0x2100,
    0x2101,
    0x2102,
    0x2103,
    0x2104,
    0x2105,
    0x2106,
    0x2107,
    0x2108,
    0x2109,
    0x210a,
    0x210b,
    0x210c,
    0x210d,
    0x210e,
    0x210f,
    0x2110,
    0x2111,
    0x2112,
    0x2113,
    0x2114,
    0x2115,
    0x2116,
    0x2117,
    0x2118,
    0x2119,
    0x211a,
    0x211b,
    0x211c,
    0x211d,
    0x211e,
    0x211f,
    0x2120,
    0x2121,
    0x2122,
    0x2123,
    0x2203,
    0x2300,
    0x2301,
    0x2303,
    0x2304,
    0x2305,
    );

my %properties;

while (<>)
{
    if (/prop: (\S+): (\S+)/)
    {
	my $prop_num = hex($1);
	my $prop_value = hex($2);
	$properties{$prop_num} = $prop_value;
    }
}

# now have all the properties in %properties
# dump the ones we are interested in

my $prop_num;

print "  { ";
foreach $prop_num (@wanted_properties)
{
    if (exists($properties{$prop_num}))
    {
	printf "0x%02x, ", $properties{$prop_num};
    }
    else
    {
	printf "not present: 0x%04x\n", $prop_num;
    }
}
print "},\n";

print "\nPut these lines in RH_RF24::setModemRegisters\n\n";
# Generate lines for RH_RF24::setModemRegisters
foreach $prop_num (@wanted_properties)
{
    printf "    set_properties(0x%04x, &config->prop_%04x, 1);\n", $prop_num, $prop_num;
}
