#!/usr/bin/perl -w

# 讀取 wosi_driver.c 產生的 ../mk-wosi.log，
# wosi_driver.c 的 TRACE 要設為 3
# 過濾掉 debug[1] (cmd_d) 為 0 的行，
# 只保留 cmd_d 不為 0 的資料，
# 寫入 ./debug.log 以做分析

use warnings;
use strict;
use Scalar::Util qw(looks_like_number);
use File::ReadBackwards;

# open (IN, "< ../mk-wosi.log")
#   || die "ERROR: no such file: ../mk-wosi.log";
my $back = File::ReadBackwards->new("../mk-wosi.log") or die $!;

open (OUT, "> debug.log")
  || die "ERROR: can not create file: debug.log";

my $n = 0;
my $m = 0;
# while (<IN>) {
while (defined($_ = $back->readline)) {
  my $line = $_;
  chomp ($line);
  
  my @fields = split(/\s+/, $line);
  my $vel_o = $fields[5];
  if (looks_like_number($vel_o)) {
    if (($vel_o >131072) or ($vel_o < -131072)) {
      $m++;
      print OUT "$line\n";
    }
  } else {
    $m++;
    print OUT "$line\n";
  }
  
  $n++;
  print "$m/$n, vel_o($vel_o)\r";
  if ($m > 10000) { # 避免產生太大的檔案
    last;
  }
}
print "\n";

close (OUT);
close (IN);
