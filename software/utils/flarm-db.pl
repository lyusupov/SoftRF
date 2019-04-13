#!/usr/bin/env perl

#
# Created by Stanislaw Pusep
#
# taken from https://gist.github.com/creaktive/34442b457ff617111844
#

use 5.010;
use strict;
use warnings qw(all);

use Carp qw(carp croak);
use Encode qw(decode);
use HTTP::Tiny;
use JSON::XS;

my $local = 'data.fln';
my $res = HTTP::Tiny
    ->new
    ->mirror('http://www.flarmnet.org/static/files/wfn/data.fln', $local);
carp "Can't mirror: $@" unless $res->{success};

my $json = JSON::XS
    ->new
    ->ascii
    ->canonical;

open(my $fh, '<', $local)
    || croak "Can't open file: $!";

# skip header
scalar <$fh>;

while (my $line = <$fh>) {
    my $plain = pack('H172', $line);
    if ($plain =~ m{^
        (?<_id>         .{6})
        (?<owner>       .{21})
        (?<airport>     .{21})
        (?<type>        .{21})
        (?<registration>.{7})
        (?<tail>        .{3})
        (?<radio>       .{7})
    $}x) {
        my %row = map {
            $+{$_} =~ m{^\s*$}
                ? ()
                : ($_ => decode latin1 => ($+{$_} =~ s{^\s+|\s+$}{}grx))
        } keys %+;
        say $json->encode(\%row);
    }
}

close $fh;