#!/usr/bin/perl -w
#-----------------------------------------------------------------------
#
#  This file is part of 34S.
#
#  34S is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  34S is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with 34S.  If not, see <http://www.gnu.org/licenses/>.
#
#-----------------------------------------------------------------------
#
my $Description = "Assembler/Disassembler for the WP 34S calculator.";
#
#-----------------------------------------------------------------------
#
# Language:         Perl script
#
my $SVN_Current_Revision  =  '$Revision: 3147 $';
#
#-----------------------------------------------------------------------

use strict;
use POSIX;
use File::Basename;

# ---------------------------------------------------------------------
# NOTE: There is a discrepancy in the way "words" and "steps" are used in the calculator
#       and I (NFH) believe these usages are incorrect and can lead to significant
#       confusion and misinterpretation.
#
#       For the purposes of this calculator, a "word" is defined as a 16-bit quantity**.
#       For storage purposes, the word is accessed in little endian mode. A word is an
#       indivisible, fundamental unit. (Bytes are only used when addressed as portions of
#       words for injection of offsets or as character encoding, or in the calculation of
#       CRCs. Thus, bytes are never communicated directly with the calculator.)
#
#       A "step" is the tokenized instruction interpreted by the calculator's execution
#       state machine. A step consists of one or two words and thus us not a fundamental
#       unit.
#
#       Several pragmas have been defined using the term "step" where they really should
#       be using the term "word". Hence, within this script, the proper term "word" will
#       be used to avoid confusion.
#
# **    It is acknowledged that the underlying ARM processor uses a 32-bit word. However,
#       the ARM is not in question in this design and, in fact, the calculator could be
#       ported from the 32-bit ARM to a 16- or 8-bit processor and still maintain
#       functionality under the current definition. Thus, it is felt that there is no
#       confusion between 16-bit vs. 32-bit words, whereas there is a significant
#       discrepancy between the usage of "steps" vs. "words" in the current definition.
#
# ---------------------------------------------------------------------
# Binary Image Specification:
#
#  NOTE: All words are defined as 16-bit and are accessed in little endian order.
#
# There are 3 (4?) image types used by various versions and sections of the calculator.
#
# Pre-V3:
#
#   In pre-V3 calculators, there are 2 images formats: RAM/flash, and XROM.
#
#   RAM/flash:
#
#   The RAM and flash images are identical. They typically had a limit of 506 words
#   per section (though this may be modified through the use of a op-code definition
#   table pragma.) They have the following format:
#
#   Word        Content
#   -----       -------------------------------
#   0           CRC covering words 1 to the last program word used.
#   1           Program token words used plus 1 (ie: next free word).
#   2--x        Program tokens.
#   (x+1)--512  Fill to 512 words using 0xFFFF.
#
#   XROM:
#
#   The XROM is not bounded by same size limitations as the RAM/flash areas. It has
#   neither the CRC nor the next free word field. It is also not blank filled to any
#   specific length. This mode must be enabled with special command line switches.
#   It has the following format:
#
#   Word    Content
#   -----   -------------------------------
#   0-?     Program tokens.
#
# NOTE: Because they are missing the size field, binary XROM images cannot be
#       disassembled using this tool.
#
# V3++:
#
#   In V3 calculators, there are 3 images formats: RAM, flash, and XROM.
#
#   RAM (aka State file):
#
#   The V3 RAM image has changed from previous versions. The 0-th word contains the
#   maximum words allowed in the region. Since the assembler cannot create a state
#   file from scratch, we don't care what gets put in this field from this tool (another
#   will fill it in). The next word contains to the number of token words actually used
#   in the region. The next area is the program token words. Undefined padding words
#   fill the spaces between the end of the program tokens and the start of the state.
#   Following the state is the CRC, which covers 0 to the end of state. Note that this
#   script doens not care about word 0, or anything after the active program tokens.
#   It has the following format:
#
#   Word      Content
#   -----     -------------------------------
#   0         Max words allowed in region
#   1         Program token words used.
#   2-x       Program tokens.
#
#   flash:
#
#   The flash image is nearly identical to the pre-V3 image with the following
#   exceptions: 1) The maximum length is always limited as per the pragma, 2) the
#   the meaning of the word in slot 1 is the last word used as opposed to the next
#   free word, and 3) it is padded out to the next 256 byte boundry (ie: 128 words).
#   This mode must be enabled with special command line switches. It has the
#   following format:
#
#   Word      Content
#   -----     -------------------------------
#   0         CRC covering words 1 to the last program word used.
#   1         Program token words used.
#   2-x       Program tokens.
#   (x+1)--Y  Fill to next multiple of 128 words using 0xFFFF.
#
#   XROM:
#
#   The XROM image is identical to the pre-V3 image with the exception that the
#   SKIP and BACK offsets are calculated differently. (Technically, this is the
#   task of the programmer or the PP tool and not the assembler. It is mentioned
#   here from reference only.) This mode must be enabled with special command line
#   switches. It has the following format:
#
#   Word    Content
#   -----   -------------------------------
#   0-?     Program tokens.
#
# ---------------------------------------------------------------------

my $debug = 0;
my $quiet = 1;
my $stderr2stdout = 0;
my (%mnem2hex, %hex2mnem, %ord2escaped_alpha, %escaped_alpha2ord);
my @files = ();

my $flash_mode = 0; # Special mode for flash libraries in V3.
my $NO_PAD_MODE = 0;
my $V2_PAD_MODE = 1;
my $V3_PAD_MODE = 2;

my $IS_NOT_AN_ALIAS = 0;
my $IS_AN_ALIAS = 1;

# Choose 0 for original, -1 for latest alias.
my $choose_disassemble_opcode = (exists $ENV{WP34S_ASM_ALIAS_DIS})
                              ? $ENV{WP34S_ASM_ALIAS_DIS}
                              : 0;

my $v3_mode = 0;
my $step_digits = 3; # Default to older style 3-digit step numbers.
my $conv_skip_back_2to3 = 0;

my $outfile = "-";
my $REDACT_COMMENTS = 1;
my $NORMAL_READ = 0;

my $DEFAULT_OP_SVN = "-- unknown --";
my $op_svn = $DEFAULT_OP_SVN;
my $report_op_svn = 1;

my $enable_pp = 0;
my $DEFAULT_PREPROC = "wp34s_pp.pl";
my $preproc = $DEFAULT_PREPROC;
my $preproc_fallback_dir = "";

# Custom arguments that can be passed to PP.
my $pp_args = "";

my $DEFAULT_PP_OPTIONS_V2 = " -m 90 "; # XXX Limit the max SKIP/BACK offset to 90 to reduce the
                                            #     likelihood of a recursive LBL insertion pushing a
                                            #     resolved SKIP/BACK beyond 99. Not a robust fix but
                                            #     likely to catch 99.99% of cases.
                                            # XXX Generate a LBL catalogue.
my $DEFAULT_PP_OPTIONS_V3 = " -m 235 "; # Same reasons, different limits.

my $pp_options = $DEFAULT_PP_OPTIONS_V2;
my $pp_listing = "wp34s_pp.lst";
my $no_skip_override = ""; # Will be set to 1 if noskip is desired.

my $PADDING_BLOCK_SIZE = 128; # XXX 128 16-bit words = 256 bytes!!
my $PADDING_FILL_VALUE = 0xFFFF;

my $FLASH_LENGTH = 512;
my $DEFAULT_FLASH_BLANK_INSTR = $PADDING_FILL_VALUE;
my $user_flash_blank_fill = "";

my $DEFAULT_OPCODE_MAP_FILE = "wp34s.op";
my $opcode_map_file = "";

# These are markers indicating the region of the opcode map where the multi-character
# opcodes live. HOPEFULLY THEY WILL ALWAYS STAY CO-LOCATED!
# These will be dynamically detected and filled in when the table is read in.
my $multi_char_target_lo = 0x0800_0000; # Initialize to bigger than it can ever be!
my $multi_char_target_hi = 0;

# If this variable is set to a non-NULL value, it will be used as the filename of the
# expanded opcode dump file. This *should* be equivalent to the "a a" file from the
# 'calc a a' program.
my $expanded_op_file = "";

# If this variable is set to a non-NULL value, it will be used as the filename of the
# dumped escaped alpha table.
my $dump_escaped_alpha_table = "";

# Number of '*' to prepend to labels in disassembly -- just makes them easier to find.
# Quoted labels will be displayed with 2x this number.
my $DEFAULT_STAR_LABELS = 2;
my $star_labels = $DEFAULT_STAR_LABELS;

my $DEFAULT_MAX_FLASH_WORDS = 506;
my $max_flash_words = $DEFAULT_MAX_FLASH_WORDS;

my $DEFAULT_MAX_RAM_WORDS = 506;
my $max_ram_words = $DEFAULT_MAX_RAM_WORDS;

my $DEFAULT_CALC_VERSION = "";
my $calc_version = $DEFAULT_CALC_VERSION;

my $disable_flash_limit = 0;

my $no_step_numbers = 0;
my $zero_last_4 = 0;

my $MAGIC_MARKER = 0xA53C;
my $CRC_INITIALIZER = 0x5aa5;
my $use_magic_marker = 0;

my $build_an_empty_image = 0;

my $ASSEMBLE_MODE = "assemble";
my $DISASSEMBLE_MODE = "disassemble";
my $DEFAULT_MODE = $ASSEMBLE_MODE;
my $mode = $DEFAULT_MODE;

# There are 2 XROM modes:
# 1) XROM output into a C array (use '-c_xrom' switch)
# 2) XROM output into a binary image (use '-xrom' switch)
my $XROM_C_MODE = 1;
my $XROM_BIN_MODE = 2;
my $xrom_leader = "#include \"xrom.h\"\nconst XROM_DATA s_opcode xrom[] = {";
my $xrom_indent_spaces = 8;
my $xrom_mode = 0;

# These are used to convert all the registers into numeric offsets.
#
# There are at least 2 groups of named instruction offsets; the REG group (XYZTABCDLIJK + .00 to .15),
# and the 104 group (ABCD). We have to extract the opcode's numeric offset based on the op-type. The
# value of max will give us this (116,112,104,100,etc.). Note that for all values of max<=100, any
# group is valid since these are purely numeric. The "stostack" group comes from the REG group.
my @reg_offset_REG = (0 .. 99, "X", "Y", "Z", "T", "A", "B", "C", "D", "L", "I", "J", "K",
                      ".00", ".01", ".02", ".03", ".04", ".05", ".06", ".07",
                      ".08", ".09", ".10", ".11", ".12", ".13", ".14", ".15");
my $MAX_REG = scalar @reg_offset_REG; # Set to length of the array.

# This is for labels and flags
my @lbl_and_flag_seed = (0 .. 99, "A", "B", "C", "D");
# Reprocess the seed array to make the numbers of the format %02d.
my (@lbl_and_flag);
foreach (@lbl_and_flag_seed) {
  if (/[A-D]/) {
    push @lbl_and_flag, $_;
  } else {
    push @lbl_and_flag, sprintf("%02d", $_);
  }
}
my $MAX_LBL_AND_FLAG = scalar @lbl_and_flag; # Set to length of the array.

# The register numeric value is flagged as an indirect reference by setting bit 7.
my $INDIRECT_FLAG = 0x80;

# XXX The calculator has difficulty with the following escaped alpha range (inclusive)
#     in the 3rd character slot of escaped alpha instructions.
my $ILLEGAL_3rd_CHAR_LO = 0xF0;
my $ILLEGAL_3rd_CHAR_HI = 0xFF;

# ANSI colour codes.
my $ansi_normal           = "\e[0m";
my $ansi_red_bg           = "\e[41;33;1m";
my $ansi_green_bg         = "\e[42;33;1m";
my $ansi_rev_green_bg     = "\e[42;1;7;33;1m";
my $ansi_rev_red_bg       = "\e[41;1;7;33;1m";
my $ansi_rev_blue_bg      = "\e[47;1;7;34;1m";
my $ansi_rev_cyan_bg      = "\e[30;46m";

my $DEFAULT_USE_ANSI_COLOUR = (exists $ENV{WP34S_ASM_COLOUR})
                            ? $ENV{WP34S_ASM_COLOUR}
                            : 0;
my $use_ansi_colour       = $DEFAULT_USE_ANSI_COLOUR;

# ---------------------------------------------------------------------

# Build a table that will translate any O/S reported to what we should be using.
# Note: At one point I thought I might need to convert cygwin to Windows type
#       so I thought I need a translation table. I worked around that but
#       will keep the table anyway because it allows me to detect O/S that
#       are "approved" with the script.
my %useable_OS = ("linux"   => "linux",
                  "MSWin32" => "MSWin32",
                  "MSDOS"   => "MSDOS",
                  "cygwin"  => "MSWin32",
                  "Darwin"  => "Darwin",
                  "darwin"  => "darwin",
                  "MacOS"   => "MacOS",
                 );

if( exists $useable_OS{$^O} ) {
  fileparse_set_fstype($useable_OS{$^O});

  # Check for OS-specific options to enable or disable.
  if ($^O =~ /MSDOS/) {
    $use_ansi_colour = 0; # Supposedly, terminal emulators under MS-DOS have a hard time running
                          # in full ANSI mode. Sigh! So, no matter what the user tries, make it
                          # difficult to turn ANSI codes on.

  # Conversely, if the OS is linux, turn colour on.
  } elsif ($^O =~ /linux/i) {
    $use_ansi_colour = 1;
  }

} else {
  # If we get here, the set file system type function has NOT been run and we may not
  # be able to locate "relative" file such as the standard external opcode table or the
  # PP script. If worst comes to worse, we can specify both of these from the command line.
  print "// WARNING: Unrecognized O/S ($^O). Some features may not work as expected.\n";
}

# Pragma table
#
# Add new variables to pragma table as required. The format is:
#                   keyword      pointer to variable
#
#                    stars    => \$star_labels,
#
# The keyword format for the wp34s.op file is starting with a letter (any case) or an underscore
# followed by zero or more letters (any case), numbers, and/or underscores. Case is significant.
# There can be 0 or more whitespace between the start of the line and/or between the "=" assignment.
# There can be 0 or 1 decimal points (sorry, no coma!) followed by 0 or more decimal digits.
#
# Examples (ignore quote):
#
# "maxsteps=510"
# "maxsteps = 510"
# "    maxsteps= 510"
# "version=30"
# "version=3.0"
#
my %pragma_table = ( maxlibsteps => \$max_flash_words,
                     maxsteps    => \$max_ram_words,
                     version     => \$calc_version,
                   );

# Automatically extract the name of the executable that was used. Also extract
# the directory so we can potentially source other information from this location.
my $script_executable = $0;
my ($script_name, $script_dir, $script_suffix) = fileparse($script_executable);

if (exists $ENV{WP34S_ASM} and ($ENV{WP34S_ASM} =~ /OS_DBG/i)) {
  debug_msg($script_name, "script_executable = '$script_executable'");
  debug_msg($script_name, "script_name       = '$script_name'");
  debug_msg($script_name, "script_dir        = '$script_dir'");
  debug_msg($script_name, "script_suffix     = '$script_suffix'");
}

if( $script_name =~ /\.exe$/i ) {
  print "// NOTE: Detected running EXE version.\n" if $debug;
  print "         Adjusting child preprocessor script name from '$preproc' to " if $debug;
  $preproc =~ s/\.pl$/\.exe/i;
  print "'$preproc'\n" if $debug;
}

my $script  = "$script_name  - $Description";
my $usage = <<EOM;

$script

Usage:
   $script_name [-pp] src_file [src_file2 [src_file3]] -o out_binary  # assembly mode
   $script_name in_binary -dis [-o out_binary] > src_file             # disassembly mode

Parameters:
   src_file         One or more WP34s program source files. Conventionally, "wp34s" is used
                    as the filename extension.
   -o outfile       Output produced by the tool. In assembler mode, this is required and will be
                    the binary flash image. Assembler output extension is conventionally ".dat".
                    In disassembler mode, this is optional and will be the output ASCII source
                    listing, conventionally uses the ".wp34s" extension. STDOUT redirection can be
                    used as an alternate method of capturing the ASCII source listing in disassembler
                    mode.
   -dis             Disassemble the binary image file.
   -pp              Launch the symbolic preprocessor ($DEFAULT_PREPROC) from within the assembler, feeding
                    the results directly back into the assembler. Only has meaning for assembly process,
                    not disassembly. Will create an intermediate file called '$pp_listing' containing
                    the post-processed source(s).
   -op infile       Optional opcode file to parse.              [Default: $DEFAULT_OPCODE_MAP_FILE]
   -s number        Optional number of asterisks (stars) to prepend to labels in
                    disassembly mode.                           [Default: $DEFAULT_STAR_LABELS]
   -syntax outfile  Turns on syntax guide file dumping. Output will be sent to 'outfile'.
   -ns              Turn off step numbers in disassembler listing.
   -dis_alias       Choose the last alias for the disassembly listing.
   -no_dis_alias    Avoid aliases for the disassembly listing.
   -no_svn          Suppress report of compressed opcode table SVN version number.
   -sb2to3          Convert old-style 2-digit SKIP/BACK offsets to 3-digit ones on the fly. Only
                    intended for use with old-style V2 programs that were not designed to take
                    advantage of PP when porting to V3. Use with discretion! (Far better to use
                    PP JMP instructions instead!)
   -h               This help script.

Examples:
  \$ $script_name great_circle.wp34s -o wp34s-3.dat
  - Assembles the named WP34s program source file producing a flash image for the WP34s.
    MUST use -o to name the output file in assembler mode -- cannot use output redirection.

  \$ $script_name  great_circle.wp34s floating_point.wp34s -o wp34s-1.dat
  - Assembles multiple WP34s program source files into a single contiguous flash image for
    the WP34s. Allows (and encourages) use of libraries of programs by concatenating the
    flash image from several source files.

  \$ $script_name -dis wp34s-1.dat -s 3 > myProg.wp34s
  \$ $script_name -dis wp34s-1.dat -s 3 -o myProg.wp34s
  - Disassembles a flash image from the WP34s. Prepend 3 asterisks to the front to each label to
    make then easier to find in the listing (they are ignored during assembly). Both invocation
    result in identical behaviour.

  \$ $script_name -pp gc_sym.wp34s vec_sym.wp34s -o wp34s-3.dat
  - Run the named file(s) through the '$DEFAULT_PREPROC' symbolic preprocessor prior to feeding the
    intermediate post-processed '$pp_listing' into the assembler proper.

  \$ $script_name -dis wp34s-0.dat -o test.wp34s; $script_name test.wp34s -o wp34s-0a.dat; diff wp34s-0.dat wp34s-0a.dat
  - An end-to-end test of the tool. Note that the blank fill mode will have to have been the same
    for the binaries to match.

Notes:
  1) Step numbers can be used in the source file but they are ignored. Since they are ignored,
     it doesn\'t matter if they are not contiguous (ie: 000, 003, 004) or not monotonic (ie: 000,
     004, 003). The disassembler does produce step numbers that are both contiguous and monotonic
     -- when not suppressed by the -ns switch.
  2) You can name a different opcode table using the -op switch. This can be used to translate a source
     written for a different SVN revision of the WP34s to move the program to a modern version of
     the WP34s. Typically, disassemble the old flash using the old opcode table and reassemble using
     the new (default) table. To generate an opcode table, using the following (Linux version shown.
     Windows is likely similar, though I have never tried):
      \$ svn up
      \$ cd ./trunk
      \$ make
      \$ ./Linux/calc opcodes > new_opcodes.map
      \$ $script_name -dis wp34s-0.dat -op new_opcodes.map > source.wp34s
  3) The prefill-value will be interpreted as decimal if it contains only decimal digits. If it
     contains any hex digits or it starts with a "0x", it will be interpreted as a hex value. Thus
     "1234" will be decimal 1234 while "0x1234" will be the decimal value 4660. Both "EFA2" and "0xEFA2"
     will be interpreted as a hex value as well (61346). The leading "0x" is optional in this case.
  4) The order the command lines switches is not significant. There is no fixed order.
  5) There is subtle difference in using the disassembler with the "-o" switch and output redirection.
     With the "-o" switch, the resulting file will not contain the commented statistics -- these will
     be displayed on the screen. With redirection, all printed lines go to the file.
EOM

my $extended_help = <<EXTENDED_HELP;
Extended parameters:
   -d level         Set the debug output verbosity level.       [Default: 0]
   -dl              Disable flash length limit.
   -c               Output the assembler result in C-array mode. Primarily for XROM assembly.
   -xrom            Assemble in XROM mode. Automatically turns off the flash length limit (-dl).
   -alpha  file     Turns on alpha table dumping. Output will be sent to 'file'.
   -empty           Used in assembler mode to generate an empty flash image. Needs '-o FILE.dat'
                    as well.
   -more_help       This extended help script.

Examples:
  \$ calc xrom | wp34s_conv.pl > xrom.wp34s
  \$ $script_name xrom.wp34s -dl -o xrom.dat
  - Assembles the XROM output into a binary image. (Mostly for test purposes.)

  \$ calc xrom | wp34s_conv.pl > xrom.wp34s
  \$ $script_name xrom.wp34s -c -o xrom.c
  - Creates a C-array of the XROM.

  \$ $script_name -syntax opcode_syntax.txt
  - Creates an opcode syntax file call 'opcode_syntax.txt' matching the current internal
    opcode table. Useful as reference when writing or transcribing source in an editor.

  \$ $script_name -empty -o wp34s-0.dat
  - Creates an empty flash image. (Could not figure out how clear an image otherwise though
    there probably is a mechanism I don\'t know about.)

Notes:
  1) XROM mode skips the first 2 words normally written to the binary image. This means that
     the CRC16 and the "next step" indicator are suppressed. This binary output is currently
     *not* able to be disassembled.
EXTENDED_HELP

#######################################################################

get_options();

# We load a compressed opcode <-> mnemonic table and expand it into an uncompressed
# version.
load_opcode_tables($opcode_map_file);

print "// Opcode SVN version: $op_svn\n" if $report_op_svn;

# Decide what value is to be used to fill the unused portion of the flash image.
# By default, the operation "ERR 03" has been used in the standard image that the
# calculator produces itself. However, there are some minor advantages to making this
# the value "FFFF". Use the '-fill <VALUE>' switch to modify the value,
my $flash_blank_fill_hex_str = ($user_flash_blank_fill) ? $user_flash_blank_fill : dec2hex4($DEFAULT_FLASH_BLANK_INSTR);

# This is where the script decides what it is going to do.
if( $build_an_empty_image ) {
  build_empty_flash_image( $outfile );
} elsif( $expanded_op_file ) {
  write_expanded_opcode_table_to_file( $expanded_op_file );
} elsif( $dump_escaped_alpha_table ) {
  write_escaped_alpha_table_to_file( $dump_escaped_alpha_table );
} elsif( $mode eq $ASSEMBLE_MODE ) {
  assemble( $outfile, @files );
} elsif( $mode eq $DISASSEMBLE_MODE ) {
  disassemble( $outfile, @files );
} else {
  die_msg(this_function((caller(0))[3]), "Internal error: Huh! How did I get here?");
}


#######################################################################
#
# Assemble the source files
#
sub assemble {
  my $outfile = shift;
  my @files = @_;
  local $_;
  my ($crc16, $alpha_text, @lines, $file);

  # We only need to prefill out the block in V2-- modes. In V3 mode, the array
  # must start from empty!
  my @words = ($v3_mode) ? () : pre_fill_flash($flash_blank_fill_hex_str); # Fill value is a hex4 string.

  my $next_free_word = 1;
  my $steps_used = 0;

  # If the preprocessor (PP) is requested, read in all the files and process them
  # as one single concatenated list of lines. This is required because the PP will
  # dynamically allocate 'local' labels as required. The PP has no concept of 'local'
  # label reuse. (XXX Maybe later.)
  if( $enable_pp ) {
    my @pp_lines = ();
    foreach $file (@files) {
      push @pp_lines, read_file($file, $NORMAL_READ);
    }
    @lines = run_pp(\@pp_lines, @files);

    # Reduce the file list to a single name -- the PP intermediate file.
    @files = ();
    push @files, $pp_listing;
  }

  # If the PP has run (above), the file list will have been changed to a single
  # file -- the PP intermediate one.
  foreach my $file (@files) {
    @lines = ();
    @lines = read_file($file, $REDACT_COMMENTS);
    @lines = conv_skip_back_v2_to_v3(@lines) if $conv_skip_back_2to3;

    for my $k (1 .. (scalar @lines)) {
      $_ = $lines[$k-1];

      next if /^\s*$/; # Skip any blank lines.
      next if /^\s*\d{3,4}\:{0,1}\s*$/; # Skip any lines that have just a 3-digit step number
                                      # and no actual mnemonic.
      s/\r//; s/\n//;

      s/\s+$//; # Remove any trailing blanks.
      my $org_line = $_;
      ($_, $alpha_text) = assembler_preformat_handling($_);

      if( not exists $mnem2hex{$_} ) {
        # If not found, try zero extending the operand and search again.
        my $zero_extend_attempts = 2;
        my $found = 0;
        while ($zero_extend_attempts) {
          $_ = zero_extend_operand($_);
          if (exists $mnem2hex{$_}) {
            $found = 1;
            last;
          }
          $zero_extend_attempts--;
        }

        # Check why we came out of the loop.
        if (not $found) {
          warn_msg(this_function((caller(0))[3]), "Cannot recognize mnemonic at line $k of file '${file}': -> ${org_line} <-");
          if( $org_line =~ /::/ ) {
            die_msg(this_function((caller(0))[3]), "There seems to be WP 34S Preprocessor labels here. Try using the '-pp' switch.\n Enter '$script_name -h' for help.");
          } else {
            die_msg(this_function((caller(0))[3]), "Cannot continue.");
          }
        }
      }

      # We will crap out before here if the instruction is not recognized.
      $steps_used++;
      # Alpha text needs to be treated separately since it encodes to 2 words.
      my $org_alpha_text = $alpha_text;
      if( length $alpha_text ) {
        # See if the text has any non ASCII characters in it. Substitute them if found.
        for( my $i = 0; $i < length $alpha_text; ++$i) {
          my $char = substr($alpha_text, $i, 1);
          if( (ord $char > 127) and (exists $escaped_alpha2ord{$char}) ) {
            $char = chr($escaped_alpha2ord{$char});
            substr($alpha_text, $i, 1) = $char;
          }
        }

        # See if the text has any escaped characters in it. Substitute them if found.
        # There may be more than one so loop until satisfied.
        while( $alpha_text =~ /(\[.+?\])/ ) {
          my $escaped_alpha = $1;
          if( exists $escaped_alpha2ord{$escaped_alpha} ) {
            my $char = chr($escaped_alpha2ord{$escaped_alpha});

            # Use a custom replacement function rather than a regex because the text
            # pattern for the substitution have regex control sequences in them and
            # that screws things up!
            $alpha_text = str_replace("${escaped_alpha}", $char, $alpha_text);
          } else {
            die_msg(this_function((caller(0))[3]), "Cannot locate escaped alpha: $escaped_alpha in table.");
          }
        }

        my @chars = split "", $alpha_text;
        debug_msg(this_function((caller(0))[3]), "number of chars: " . scalar @chars) if $debug gt 1;
        $words[++$next_free_word] = hex2dec($mnem2hex{$_}) | ord($chars[0]);

        # Quoted alpha slots have room for 3 characters -- 1 sits with the main op-code,
        # and 2 more are in the next words. We can have any combination of 1, 2, or 3
        # characters. Any character slot that is not used is nulled out.
        #
        # Characters are present in all three of the quote slots -- 1 in the main slot
        # and 2 in the secondary slot.
        if( ((scalar @chars) > 2) and length $chars[1] and length $chars[2] ) {
          # XXX Due to a bug discovered by Pauli, the calculator misbehaves when these characters
          #     are in the 3rd character slot. To work around this, we will limit them to not being
          #     allowed in that slot.
          my $ord = ord($chars[2]);
          if( ($ord >= $ILLEGAL_3rd_CHAR_LO) and ($ord <= $ILLEGAL_3rd_CHAR_HI) ) {
            my @esc_alpha_array = @{$ord2escaped_alpha{$ord}};
            my $bad = $esc_alpha_array[0];
            die_msg(this_function((caller(0))[3]), "Cannot use this escaped alpha $bad ($ord) in 3rd character slot in step '$org_line'.");
          }
          $words[++$next_free_word] = ord($chars[1]) | (ord($chars[2]) << 8);

        # Characters are present in just 2 of the 3 quote slots -- ie: 1 in the main
        # slot and only 1 of these 2.
        } elsif( ((scalar @chars) > 1) and length $chars[1] ) {
          $words[++$next_free_word] = ord($chars[1]);

        # No characters are present in final 2 quote slots. Just blank it out.
        } else {
          $words[++$next_free_word] = 0;
        }

      # "Normal" opcodes...
      } else {
        $words[++$next_free_word] = hex2dec($mnem2hex{$_});
      }

      # Lib-mode and RAM/State-mode (ie: not $flash_mode) have different size limitations.
      # Calculate if the limit is exceeded accordingly.
      if ($flash_mode and ($next_free_word >= $max_flash_words)) {
        die_msg(this_function((caller(0))[3]), "Too many program steps encountered in lib-mode (> $max_flash_words words).");
      } elsif (not $flash_mode and ($next_free_word >= $max_ram_words) and not $disable_flash_limit) {
        die_msg(this_function((caller(0))[3]), "Too many program steps encountered (> $max_ram_words words).");
      }
    }
  }

  # We have 3 output modes:
  #  - A C-array for use with XROM
  #  - An XROM binary with no output of the CRC or step count.
  #  - Standard assembler binary output.
  if( $xrom_mode == $XROM_C_MODE ) {
    dump_c_array( $outfile, $xrom_leader, $xrom_indent_spaces, @words[2 .. $next_free_word] );

  } elsif( $xrom_mode == $XROM_BIN_MODE ) {
    $crc16 = calc_crc16( @words[2 .. $next_free_word] );
    write_binary( $outfile, $next_free_word, @words[2 .. $next_free_word] ); # Limit the words that are processed.
    print "// XROM mode -- NOTE: Currently cannot be disassembled.\n";
    print "// CRC16: ", dec2hex4($crc16), "\n";
    print "// Total words: ", $next_free_word-1, "\n";
    print "// Total steps: $steps_used\n";

  } elsif ($v3_mode) {
    $words[1] = $next_free_word - 1;
    $crc16 = ($use_magic_marker) ? $MAGIC_MARKER : calc_crc16( @words[2 .. $next_free_word] );
    if (not $flash_mode) {
      $words[0] = $max_ram_words;
      push @words, $crc16; # Place CRC after everything else.
      write_binary( $outfile, $next_free_word+1, $NO_PAD_MODE, @words ); # Need to extend to include newly appended CRC.
    } else {
      $words[0] = $crc16;
      write_binary( $outfile, $next_free_word, $V3_PAD_MODE, @words );
    }
    print "// WP 34s version: $calc_version\n" if $calc_version;
    print "// CRC16: ", dec2hex4($crc16), "\n";

    if (not $flash_mode) {
      print "// Running in V3 State-mode. Max words: $max_ram_words\n";
    } else {
      print "// Running in V3 Flash-mode. Max words: $max_flash_words\n";
    }
    print "// Total words: ", $next_free_word-1, "\n";
    print "// Total steps: $steps_used\n";

  # V2-- mode
  } else {
    $words[1] = $next_free_word;
    $crc16 = ($use_magic_marker) ? $MAGIC_MARKER : calc_crc16( @words[2 .. $next_free_word] );
    $words[0] = $crc16;
    write_binary( $outfile, $next_free_word, $V2_PAD_MODE, @words );
    print "// WP 34s version: $calc_version\n" if $calc_version;
    print "// CRC16: ", dec2hex4($crc16), "\n";
    print "// Running in RAM-mode. RAM-mode max words: $max_ram_words\n";
    print "// Total words: ", $next_free_word-1, "\n";
    print "// Total steps: $steps_used\n";
  }

  return;
} # assemble


#######################################################################
#
# Disassemble the source files
#
sub disassemble {
  my $outfile = shift;
  my @files = @_;

  open OUT, "> $outfile" or die_msg(this_function((caller(0))[3]), "Cannot open file '$outfile' for writing: $!");
  print "// $script_name: Source file(s): ", join (", ", @files), "\n";

  foreach my $file (@files) {
    my $double_words = 0;
    open DAT, $file or die_msg(this_function((caller(0))[3]), "Cannot open dat file '$file' for reading: $!");

    # This trick will read in the binary image in 16-bit words of the correct endian.
    local $/;
    my @words = unpack("S*", <DAT>);

    # In V3 mode, the length parameter does not need to be decremented, as it
    # does in V2-- mode.
    my $len = ($v3_mode) ? $words[1] : ($words[1] - 1);

    my $total_steps = 0;
    for( my $k = 0; $k < $len; $k++ ) {
      my $word = $words[$k+2];
      $total_steps++;

      # Check if we are in the 2 words opcode range. These require special treatment.
      if( ($word >= $multi_char_target_lo) and ($word < ($multi_char_target_hi + 256)) ) {
        my $base_op = $word & 0xFF00;
        my (@chars);
        $chars[0] = $word & 0xFF;
        $word = $words[$k+3];
        $double_words++;
        ($chars[2], $chars[1]) = (($word >> 8), ($word & 0xFF));
        print_disassemble_text($total_steps, $base_op, @chars);
        $k++;
      } else {
        print_disassemble_normal($total_steps, $word);
      }
    }
    print "// $total_steps total instructions used.\n";
    print "// $len total words used.\n";
    print "// ", $len - (2*$double_words), " single word instructions.\n";
    print "// $double_words double word instructions.\n";
    close DAT;
  }
  close OUT;
  return;
} # disassemble


#######################################################################
#
# Print the lines which do not include quoted characters.
#
sub print_disassemble_normal {
  my $idx = shift;
  my $word = shift;
  my $hex_str = lc dec2hex4($word);
  if( not exists $hex2mnem{$hex_str} ) {
    my $msg = "Opcode '$hex_str' does not exist at line " . $idx+1 . " at print_disassemble_normal";
    die_msg(this_function((caller(0))[3]), "$msg");
  }

  # Select the mnemic to display.
  my $active_mnem = ${$hex2mnem{$hex_str}}[$choose_disassemble_opcode];

  # Set up the correct number of leading asterisks if requested.
  my $label_flag = (($active_mnem =~ /LBL/) and $star_labels) ? "*" x $star_labels : "";


  if( $no_step_numbers ) {
    printf OUT "%0s%0s\n", $label_flag, $active_mnem;
  } elsif( $debug ) {
    printf OUT "%0${step_digits}d /* %04s */ %0s%0s\n", $idx, $hex_str, $label_flag, $active_mnem;
  } else {
    printf OUT "%0${step_digits}d %0s%0s\n", $idx, $label_flag, $active_mnem;
  }
  return;
} # print_disassemble_normal


#######################################################################
#
# Print the lines that include up to 3 quoted characters For example "LBL'ABC'".
# Substitute the characters which exist in the escaped-alpha translation
# table with their escaped counterparts. For example, the 2-word opcode
# "12a3 0058" will get translated to "LBL'[delta]X'".
#
sub print_disassemble_text {
  my $idx = shift;
  my $opcode = shift;
  my @chars = @_;
  my $hex_str = lc dec2hex4($opcode);
  if( not exists $hex2mnem{$hex_str} ) {
    my $msg = "Opcode '$hex_str' does not exist at line " . $idx+1 . " at print_disassemble_text";
    die_msg(this_function((caller(0))[3]), "$msg");
  }

  # Select the mnemic to display.
  my $active_mnem = ${$hex2mnem{$hex_str}}[$choose_disassemble_opcode];

  # Set up the correct number of leading asterisks if requested.
  my $label_flag = (($active_mnem =~ /LBL/) and $star_labels) ? "*" x (2 * $star_labels) : "";

  if( $no_step_numbers ) {
    printf OUT "%0s%0s", $label_flag, $active_mnem;
  } elsif( $debug ) {
    my $op_hex = sprintf "%04s %04s", lc dec2hex4(hex2dec($hex_str)+$chars[0]), lc dec2hex4($chars[2]*256+$chars[1]);
    printf OUT "%0${step_digits}d /* %04s */ %0s%0s", $idx, $op_hex, $label_flag, $active_mnem;
  } else {
    printf OUT "%0${step_digits}d %0s%0s", $idx, $label_flag, $active_mnem;
  }

  # Check to see if the character is an escaped-alpha. If so, print the escaped string
  # rather than the actual characters.
  foreach my $ord (@chars) {
    if( exists $ord2escaped_alpha{$ord} ) {
      my $escaped_alpha = $ord2escaped_alpha{$ord}[$choose_disassemble_opcode];
      print OUT "${escaped_alpha}" if $ord;
    } else {
      print OUT chr($ord) if $ord;
    }
  }
  print OUT "'\n"; # Append the trailing single quote.
  return;
} # print_disassemble_normal


#######################################################################
#
# Compute a CRC16 over the stream of bytes. We have convert the word array to
# a byte array to do this.
#
sub calc_crc16 {
  my @byteArray = wordArray2byteArray(@_);
  my $crc = $CRC_INITIALIZER;
  foreach (@byteArray) {
    $crc = (($crc >> 8) & 0xFFFF) | (($crc << 8) & 0xFFFF);
    $crc ^= $_;
    $crc ^= ($crc & 0xFF) >> 4;
    $crc ^= ($crc << 12) & 0xFFFF;
    $crc ^= (($crc & 0xFF) << 5) & 0xFFFF;
  }
  return $crc;
} # calc_crc16


#######################################################################
#
#
#
sub wordArray2byteArray {
  my @wordArray = @_;
  my (@byteArray);
  local $_;
  foreach (@wordArray) {
    push @byteArray, ($_ & 0xFF); # Low byte
    push @byteArray, (($_ >> 8) & 0xFF); # High byte
  }
  return @byteArray;
} # wordArray2byteArray


#######################################################################
#
# Pad mode:
#   NO_PAD_MODE:  Don't add any padding.
#   V2_PAD_MODE:  Padding already in place. (Actually identical to NO_PAD_MODE from
#                 this function's point-of-view).
#   V3_PAD_MODE:  Pad to next 256-byte boundary.
#
sub write_binary {
  my $outfile = shift;
  my $last_word = shift;
  my $pad_mode = shift;
  my @words = @_;

  open OUT, "> $outfile" or die_msg(this_function((caller(0))[3]), "Cannot open file '$outfile' for writing: $!");
  binmode OUT;

  if ($pad_mode == $V3_PAD_MODE) {
    # In V3 mode, we need to pad out to a multiple of a specific pad block size.
    # Unlike V2-- modes, V3 mode's array is not prepadded to a specific block size. The array
    # has only real data in it and no more. Threfore, the array needs to be padded out
    # to the defined block size.
    for( my $k = $last_word + 1; ((scalar(@words) % $PADDING_BLOCK_SIZE) != 0); $k++ ) {
      $words[$k] = $PADDING_FILL_VALUE;
    }
  }

  foreach (@words) {
    my $bin_lo = $_ & 0xFF;
    my $bin_hi = $_ >> 8;
    print OUT chr($bin_lo), chr($bin_hi);
  }
  close OUT;
  return;
} # write_binary


#######################################################################
#
# Handle any lines that need special work. If the opcode has single
# quotes in it ('xxx'), it needs to have the alpha string extracted to the
# "$alpha_text" variable. If "$alpha_text" is returned as the NULL string,
# the opcode was not of this type. Thus, this "$alpha_text" flag is used
# above to signal the assembler to switch into the 2-word opcode translation
# mode required for these quoted alpha mnemonics.
#
sub assembler_preformat_handling {
  my $line = shift;
  my $alpha_text = "";

  $line =~ s/^\s*\d{3,4}\:{0,1}\s+//; # Remove any line numbers with or without a colon
  $line =~ s/^\s+//;      # Remove leading whitespace
  $line =~ s/\s+$//;      # Remove trailing whitespace

  # Labels are sometimes displayed with an asterisk to make them easier to locate.
  # Remove these at this time.
  $line =~ s/^\*+LBL/LBL/;

  # Remove the actual label string so we can do a lookup based on the base mnemonic.
  if( $line =~ /\S+\'(\S+)\'/ ) {
    $alpha_text = $1;
    $line =~ s/\'\S+/\'/;
    debug_msg(this_function((caller(0))[3]), "text: '$alpha_text'") if $debug gt 1;
  }

  return ($line, $alpha_text);
} # assembler_preformat_handling


#######################################################################
#
# Prefill an array with an empty flash image.
#
sub pre_fill_flash {
  my $fill_value = shift;
  my $start = 0;
  my $length = $FLASH_LENGTH;
  my (@flash);
  for(my $k = 0; $k < $FLASH_LENGTH; $k++ ) {
    $flash[$k] = hex2dec($fill_value);
  }

  # Overwrite the 'special' locations.
  $flash[0] = $MAGIC_MARKER;
  $flash[1] = 1;
  # If strict image compatibility is required, zero these words.
  if( $zero_last_4 ) {
    $flash[-1] = 0;
    $flash[-2] = 0;
    $flash[-3] = 0;
    $flash[-4] = 0;
  }
  return @flash;
} # pre_fill_flash


#######################################################################
#
# Load the opcode tables.
#
sub load_opcode_tables {
  my $file = shift;

  # If the user has specified an opcode table via the -opcodes switch, use that one.
  if( $file ) {
    open DATA, $file or die_msg(this_function((caller(0))[3]), "Cannot open opcode map file '$file' for reading: $!");
    $file .= " (specified)";

  } else {
    # Search the current directory for an opcode table.
    if( -e "${DEFAULT_OPCODE_MAP_FILE}" ) {
      $file = "${DEFAULT_OPCODE_MAP_FILE}";
      open DATA, $file or die_msg(this_function((caller(0))[3]), "Cannot open opcode map file '$file' for reading: $!");
      $file .= " (local directory)";

    # Search the directory the script is running out of.
    } elsif( -e "${script_dir}${DEFAULT_OPCODE_MAP_FILE}" ) {
      $file = "${script_dir}${DEFAULT_OPCODE_MAP_FILE}";
      open DATA, $file or die_msg(this_function((caller(0))[3]), "Cannot open opcode map file '$file' for reading: $!");

    } else {
      die_msg(this_function((caller(0))[3]), "Cannot locate op-code table '$DEFAULT_OPCODE_MAP_FILE' in either current directory or '${script_dir}'.");
    }
  }
  print "// Opcode map source: $file\n";

  # Read from whatever source happened to be caught. If none of the above opened
  # a file, read the attached DATA segment at the end of the script as the fallback.
  while(<DATA>) {
    # Extract the SVN version of the opcode table -- if it exists.
    if( /^\s*#\s+Generated .+ svn_(\d+)\.op/ ) {
      $op_svn = $1;

    # Alternately, look for a stock SVN Rev keyword substitution.
    # Take care to ignore the case just in case it is REV instead of Rev.
    } elsif( /^\s*#\s+\$Rev:\s+(\d+)\s*\$/i ) {
      $op_svn = $1;
    }

    # Remove any comments ('#' or '//') and any blank lines.
    next if /^\s*#/;
    next if /^\s*$/;
    next if m/^\s*\/\//;
    chomp; chomp;

    # Annihilate every end-of-line known to mankind! The "chomp; chomp;" doesn't
    # seem to work for lines ending in 0x0D/0x0A (Why not? I thought that is what it
    # did!). So take extreme measures and scrub everything that could be remotely
    # construed as an EOL!
    s/\r//;
    s/\n//;

    # From about WP34s SVN 1000 on, the console can spit out a "compressed" opcode map
    # when run using: "./trunk/Linux/calc opcodes > svn_XXXX.op"
    #
    # From Paul Dale:
    #
    # Fields are tab separated.
    #
    # The first field is the numeric op-code in hex.
    #
    # The second field is the type of op-code. cmd is a standard command. mult is a
    # multi-character alpha command (double length). arg is a command that takes an
    # argument. alias-c, alias-m and alias-a are alias names for the command types
    # listed above.
    #
    # The third field is the op-code itself.
    #
    # For argument commands there is a fourth field that can contain up to four pieces
    # of information comma separated:
    #
    # 'max=nnn' nnn is the maximum value + 1 permitted as an argument. I.e. legal arguments
    # are 0 through nnn-1 inclusive.
    #
    # 'indirect' is present if the command is permitted an indirect argument. If it is, the
    # argument can of course be any register including the stack (0 - 111 inclusive).
    #
    # 'stack' is present if a stack register is allowed as an argument.
    #
    # 'complex' is present if the argument specifies a complex pair of registers. This
    # basically restricts numeric registers to be 0 - 98 instead of 0 - 99 and it restricts
    # lettered registers to be X, Z, A, C, L and J.
    #

    # See if there is an XROM tag on the instruction. If so, it is ONLY valid when running in
    # an XROM mode. If we are not in xrom mode, simply remove the instruction from consideration.
    # If this instruction is used when we are not in XROM mode, an error will be generated
    # indicating the instruction is invalid -- which is true in non-XROM modes. No need to
    # elaborate to the user that the instruction is only valid in XROM mode.
    if (not $xrom_mode) {
      next if /\,xrom/ or /\s+xrom/; # Simply skip the instructions.
    } else {
      # If we are in XROM mode, scrub the tag from the line prior to normal processing.
      # There are 2 slightly different forms and we need to deal with both.
      s/\,xrom//;
      s/\s+xrom//;
    }

    # cmd-type line
    if( /0x([0-9a-fA-F]{4})\s+cmd\s+(.+)$/ ) {
      my $hex_str = $1;
      my $mnemonic = $2;
      my $line_num = $.;

      if ($mnemonic eq "END") {
        $v3_mode = 1;
        $pp_options = $DEFAULT_PP_OPTIONS_V3;
        $step_digits = 4;
      }

      load_table_entry($hex_str, $mnemonic, $line_num, $IS_NOT_AN_ALIAS);

      # We need to harvest the escaped-alpha table for later substitutions.
      if( /\[alpha\]\s+(\[.+\])/ ) {
        my $escaped_alpha = $1;
        load_escaped_alpha_tables($hex_str, $escaped_alpha, $line_num, $IS_NOT_AN_ALIAS);
      }

      # As a convenience, create a secondary entry for iC commands that have a 2-digit
      # number. This form can be used instead of entering the longer constant form. For
      # example, "iC 03" will be accepted as well as "iC 5.01402". Note that the offset
      # will are in decimal so "iC 12" is treated as "iC 0.56275713" and not "iC 0.03255816",
      # which would have been the hex offset of 0x12.
      if( $mnemonic =~ /iC/ ) {
        load_2digit_iC_entry($hex_str, $mnemonic, $line_num);
      }

    # alias to the above
    } elsif( /0x([0-9a-fA-F]{4})\s+alias-c\s+(.+)$/ ) {
      my $hex_str = $1;
      my $mnemonic = $2;
      my $line_num = $.;
      load_table_entry($hex_str, $mnemonic, $line_num, $IS_AN_ALIAS);

      # We need to harvest the escaped-alpha table for later substitutions.
      if( /\[alpha\]\s+(\[.+\])/ ) {
        my $escaped_alpha_alias = $1;
        load_escaped_alpha_tables($hex_str, $escaped_alpha_alias, $line_num, $IS_AN_ALIAS);
      } elsif( /\[alpha\]\s+(.)/ ) {
        my $alpha_alias = $1;
        if( ord($alpha_alias) > 127 ) {
          load_escaped_alpha_tables($hex_str, $alpha_alias, $line_num, $IS_AN_ALIAS);
        }
      }


    # arg-type line
    } elsif( /0x([0-9a-fA-F]{4})\s+arg\s+(.+)$/ ) {
      my $hex_str = $1;
      my $parameter = $2;
      my $line_num = $.;
      parse_arg_type($hex_str, $parameter, $line_num, $IS_NOT_AN_ALIAS);

    # alias to the above
    } elsif( /0x([0-9a-fA-F]{4})\s+alias-a\s+(.+)$/ ) {
      my $hex_str = $1;
      my $parameter = $2;
      my $line_num = $.;
      parse_arg_type($hex_str, $parameter, $line_num, $IS_AN_ALIAS);

    # mult-type line
    } elsif( /0x([0-9a-fA-F]{4})\s+mult\s+(.+)$/ ) {
      my $hex_str = $1;
      my $mnemonic = "${2}'"; # append a single quote.
      my $line_num = $.;
      load_table_entry($hex_str, $mnemonic, $line_num, $IS_NOT_AN_ALIAS);

      # Dynamically extract the band of opcodes that require special processing for
      # multi-character strings.
      # The lower band...
      if( hex2dec($hex_str) < $multi_char_target_lo ) {
        my $msg = "Replacing the multi-char lower limit using '$mnemonic'. Was: " . dec2hex4($multi_char_target_lo) . ", Now is: $hex_str";
        debug_msg(this_function((caller(0))[3]), $msg) if $debug > 2;
        $multi_char_target_lo = hex2dec($hex_str);
      }

      # The upper band... Note that this will be offset by 256 when used to account for
      # the offset byte within the opcode.
      if( hex2dec($hex_str) > $multi_char_target_hi ) {
        my $msg = "Replacing the multi-char upper limit using '$mnemonic'. Was: " . dec2hex4($multi_char_target_hi) . ", Now is: $hex_str";
        debug_msg(this_function((caller(0))[3]), $msg) if $debug > 2;
        $multi_char_target_hi = hex2dec($hex_str);
      }

    # alias of the above
    } elsif( /0x([0-9a-fA-F]{4})\s+alias-m\s+(.+)$/ ) {
      my $hex_str = $1;
      my $mnemonic = "${2}'"; # append a single quote.
      my $line_num = $.;
      load_table_entry($hex_str, $mnemonic, $line_num, $IS_AN_ALIAS);

    # Parse any of the pragma-like values.
    } elsif( /^\s*([a-zA-Z_][a-zA-Z0-9_]*)\s*\=\s*(\d+\.{0,1}\d*)\s*$/ ) {
      my $variable = $1;
      my $value = $2;

      # Just ignore anything that isn't found in the pragma table. If it is found,
      # dereference the pointer from the hash table and assign the new value.
      if( exists $pragma_table{$variable} ) {
        ${$pragma_table{$variable}} = $value;
      }

    } else {
      die_msg(this_function((caller(0))[3]), "Cannot parse opcode table line $.: '$_'");
    }
  }
  close DATA;
  return;
} # load_opcode_tables


#######################################################################
#
# Write the expanded opcode table to a file.
#
sub write_expanded_opcode_table_to_file {
  my $file = shift;
  local $_;
  open EXPND, "> $file" or die_msg(this_function((caller(0))[3]), "Cannot open expanded opcode file '$file' for writing: $!");
  print EXPND "// Opcode SVN version: $op_svn\n";

  # Print an entry for each potential alias in the op-code table.
  for my $hex_str (sort keys %hex2mnem) {
    my $array_pntr = $hex2mnem{$hex_str};
    my @mnems = @{$array_pntr};
    foreach my $mnem (@mnems) {
      print EXPND "$hex_str  $mnem\n";
    }
  }
  close EXPND;
  return;
} # write_expanded_opcode_table_to_file


#######################################################################
#
# Write the escaped alpha table to a file.
#
sub write_escaped_alpha_table_to_file {
  my $file = shift;
  local $_;
  open ALPHA, "> $file" or die_msg(this_function((caller(0))[3]), "Cannot open escaped alpha file '$file' for writing: $!");
  for my $ord (sort {$a <=> $b} keys %ord2escaped_alpha) {
    print ALPHA "ord = $ord (decimal), 0x", dec2hex2($ord), "(hex); alpha = '${ord2escaped_alpha{$ord}}'\n";
  }
  close ALPHA;
  return;
} # write_escaped_alpha_table_to_file


#######################################################################
#
# Parse the arg-type
#
# For argument commands there is a fourth field that can contain up to four pieces
# of information comma separated:
#
# 'max=nnn' nnn is the maximum value + 1 permitted as an argument. I.e. legal arguments
# are 0 through nnn-1 inclusive.
#
# 'indirect' is present if the command is permitted an indirect argument. If it is, the
# argument can of course be any register including the stack (0 - 111 inclusive).
#
# 'stack' is present if a stack register is allowed as an argument.
#
# 'stostack' is present if the registers are limited to 00-95 and A.
#
# 'complex' is present if the argument specifies a complex pair of registers. This
# basically restricts numeric registers to be 0 - 98 instead of 0 - 99 and it restricts
# lettered registers to be X, Z, A, C, L and J.
#
sub parse_arg_type {
  my $base_hex_str = shift;
  my $arg = shift;
  my $line_num = shift;
  my $is_alias = shift;

  # Leave in the code until we get the kinks worked out.
  if (not defined $is_alias) {
    warn_msg(this_function((caller(0))[3]), "Developement bug: \$is_alias is undefined");
  }

  my ($base_mnemonic);
  my $direct_max = 0;
  my $indirect_modifier = 0;
  my $stackreg_modifier = 0;
  my $stostack_modifier = 0;

  if( $arg =~ /(\S+)\s+/ ) {
    $base_mnemonic = $1;
  } else {
    die_msg(this_function((caller(0))[3]), "Cannot parse base mnemonic from arg-type line $line_num: '$arg'");
  }

  if( $arg =~ /max=(\d+)/ ) {
    $direct_max = $1;
  } else {
    die_msg(this_function((caller(0))[3]), "Cannot parse max parameter from arg-type line $line_num: '$arg'");
  }

  $indirect_modifier = 1 if $arg =~ /indirect/;
  $stackreg_modifier = 1 if $arg =~ /stack/; # We allow the stostack match here because the format is the same.
  $stostack_modifier = 1 if $arg =~ /stostack/;

  if( $indirect_modifier and ($direct_max > 128) ) {
    # Limit max to 128 if we are in an indirect-applicable opcode. LOCAL seems to be in effect.
    # Shouldn't happen any more.
    $direct_max = 128;
  }

  # Load the direct argument variants
  if( $direct_max ) {
    for my $offset (0 .. ($direct_max - 1)) {
      if( ! $stostack_modifier || ($offset <= 96) || ($offset >= 112) ) {
        parse_arg_type_dir_max($direct_max, $offset, $base_mnemonic, $base_hex_str, $line_num,
                               $indirect_modifier, $stackreg_modifier, $is_alias);
      }
    }
  }

  # Load the indirect argument variants. These always comes from the REG set.
  if( $indirect_modifier ) {
    for my $offset (0 .. ($MAX_REG - 1)) {
      # "Correct" the format if it is in the numeric range [0-99].
      my $reg_str = ($offset < 100) ? sprintf("%02d", $offset) : $reg_offset_REG[$offset];
      my $indirect_offset = $offset + $INDIRECT_FLAG;
      my $mnemonic_str = "$base_mnemonic";
      $mnemonic_str .= "[->]";
      my ($hex_str, $mnemonic) = construct_offset_mnemonic($base_hex_str, $indirect_offset, $mnemonic_str, $reg_str);
      load_table_entry($hex_str, $mnemonic, $line_num, $is_alias);

      # Construct aliases. Hex value remains the same for all sets.
      # Note that any additional entries we add here are aliases by definition so
      # flag them a such to avoid the replication warning checks.
      my @indirection_aliases = ("=>", "->", ">", "@"); # Be generous in what we accept!
      foreach my $indirection_alias (@indirection_aliases) {
        $mnemonic = "${base_mnemonic} ${indirection_alias}${reg_str}";
        load_table_entry($hex_str, $mnemonic, $line_num, $IS_AN_ALIAS);
      }
    }
  }

  return;
} # parse_arg_type


#######################################################################
#
#
#
sub parse_arg_type_dir_max {
  my $direct_max = shift;
  my $offset = shift;
  my $base_mnemonic = shift;
  my $base_hex_str = shift;
  my $line_num = shift;
  my $indirect_modifier = shift;
  my $stackreg_modifier = shift;
  my $is_alias = shift;

  # Leave in the code until we get the kinks worked out.
  if (not defined $is_alias) {
    warn_msg(this_function((caller(0))[3]), "Developement bug: \$is_alias is undefined");
  }

  my ($reg_str);

  # Find out which instruction offset group we are to use.
  if ($direct_max <= 10) {
    $reg_str = $offset;
  } elsif ($direct_max == $MAX_LBL_AND_FLAG) {
    $reg_str = $lbl_and_flag[$offset];
  } elsif ($direct_max <= 100 or $stackreg_modifier) {
    $reg_str = sprintf("%02d", $offset)
  } else {
    $reg_str = sprintf("%03d", $offset)
  }
  if ($offset >= 100) {
    $reg_str = $reg_offset_REG[$offset] if ($stackreg_modifier);
  }
  $reg_str = "--UNDEFINED--" if not defined $reg_str;

  my ($hex_str, $mnemonic) = construct_offset_mnemonic($base_hex_str, $offset, "$base_mnemonic ", $reg_str);
  load_table_entry($hex_str, $mnemonic, $line_num, $is_alias);
  return;
} # parse_arg_type_dir_max


#######################################################################
#
#
#
sub load_escaped_alpha_tables {
  my $hex_str = shift;
  my $escaped_alpha = shift;
  my $line_num = shift;
  my $is_alias = shift;

  # Leave in the code until we get the kinks worked out.
  if (not defined $is_alias) {
    warn_msg(this_function((caller(0))[3]), "Developement bug: \$is_alias is undefined");
  }

  my $ord = hex2dec($hex_str) & 0xFF;
  my (@esc_alpha_array);

  # Build the table to convert ordinals to escaped alpha, eg: the entry for 0x1D (ie: 29 decimal)
  # would be an array containing at least "[approx]" (at least for some -- possibly distant past
  # -- op-code table). There may be additional aliases for the 0x1D code in the array as well.
  # These will be added to the array contained in the hash.
  if( not exists $ord2escaped_alpha{$ord} ) {
    push @esc_alpha_array, $escaped_alpha; # Add the initial (base) representation.
    $ord2escaped_alpha{$ord} = \@esc_alpha_array; # ... and save the array in the hash.
  } else {
    if ($is_alias == $IS_NOT_AN_ALIAS) {
      my $msg = "Duplicate ordinal for escaped alpha '$escaped_alpha': $ord (0x" . dec2hex2($ord) . ") at line $line_num";
      warn_msg(this_function((caller(0))[3]), $msg) unless $quiet;
    } else {
      @esc_alpha_array = @{$ord2escaped_alpha{$ord}}; # Recover the existing array to add a new entry.
      push @esc_alpha_array, $escaped_alpha; # Add the new alias.
      $ord2escaped_alpha{$ord} = \@esc_alpha_array; # Replace the array currently in the hash.
    }
  }

  # Build the table to convert escaped alpha to ordinals, eg: the entry for "[approx]" would be
  # 0x1D (ie: 29 decimal) -- (at least for some -- possibly distant past -- op-code table).
  if( not exists $escaped_alpha2ord{$escaped_alpha} ) {
    $escaped_alpha2ord{$escaped_alpha} = $ord;
  } else {
    warn_msg(this_function((caller(0))[3]), "Duplicate escaped alpha: '$escaped_alpha' at line $line_num") unless $quiet;
  }
  return;
} # load_escaped_alpha_tables


#######################################################################
#
# Make a duplicate entry for the "iC' mnemonics to allow a 2-digit offsets version of
# the mnemonic. Only load this the mnemonic to hex table to avoid multiple definitions
# in the hex to mnemonic table.
#
sub load_2digit_iC_entry {
  my $hex_str = shift;
  my $mnemonic = shift;
  my $line_num = shift;
  my $dup_mnemonic = sprintf("iC %02d", hex2dec($hex_str) & 0xFF);
  load_mnem2hex_entry($hex_str, $dup_mnemonic, $line_num);
  return;
} # load_2digit_iC_entry


#######################################################################
#
# Build an entry pair similar to the older format which includes the opcode (in a hex
# string) offset by the correct amount and the reconstructed mnemonic name.
#
sub construct_offset_mnemonic {
  my $base_hex_str = shift;
  my $offset = shift; # Decimal
  my $mnemonic = shift;
  my $reg_name_str = shift;
  return (dec2hex4(hex2dec($base_hex_str) + $offset), "${mnemonic}${reg_name_str}");
} # construct_offset_mnemonic


#######################################################################
#
# Load the translation hash tables with opcode (as a hex string) and the mnemonic.
#
sub load_table_entry {
  my $op_hex_str = shift;
  my $mnemonic = shift;
  my $line_num = shift;
  my $is_alias = shift;

  # Leave in the code until we get the kinks worked out.
  if (not defined $is_alias) {
    warn_msg(this_function((caller(0))[3]), "Developement bug: \$is_alias is undefined");
  }

  debug_msg(this_function((caller(0))[3]), "Loading entry ${line_num} ${op_hex_str} ${mnemonic}") if $debug > 1;
  load_mnem2hex_entry($op_hex_str, $mnemonic, $line_num);
  load_hex2mnem_entry($op_hex_str, $mnemonic, $line_num, $is_alias);
  return;
} # load_table_entry


#######################################################################
#
# Load the translation hash table with opcode (as a hex string) and the mnemonic.
#
sub load_mnem2hex_entry {
  my $op_hex_str = shift;
  my $mnemonic = shift;
  my $line_num = shift;

  if( not exists $mnem2hex{$mnemonic} ) {
    $mnem2hex{$mnemonic} = lc $op_hex_str;
  } else {
    warn_msg(this_function((caller(0))[3]), "Duplicate mnemonic: '$mnemonic' at line $line_num (new definition: '$op_hex_str', previous definition: '${mnem2hex{$mnemonic}}')");
  }
  return;
} # load_mnem2hex_entry


#######################################################################
#
# Load the translation hash tables with opcode (as a hex string) and the mnemonic.
# The mnemonic is stored in an array in order to account for aliases for the op-code.
#
sub load_hex2mnem_entry {
  my $op_hex_str = shift;
  my $mnemonic = shift;
  my $line_num = shift;
  my $is_alias = shift;

  # Leave in the code until we get the kinks worked out.
  if (not defined $is_alias) {
    warn_msg(this_function((caller(0))[3]), "Developement bug: \$is_alias is undefined");
  }

  my (@mnem_array);

  # If this is the first entry for this op-code, create a new array to load the options.
  if( not exists $hex2mnem{lc $op_hex_str} ) {
    push @mnem_array, $mnemonic;
    $hex2mnem{lc $op_hex_str} = \@mnem_array;

  # If this is not the first entry for this op-code, it could be either an duplication error or an alias.
  # If the former, generate a warning. If the latter, add a new entry to the array.
  } else {
    if ($is_alias == $IS_NOT_AN_ALIAS) {
      my $prev_defs = join ", ", @{$hex2mnem{lc $op_hex_str}};
      warn_msg(this_function((caller(0))[3]), "Duplicate opcode hex: '$op_hex_str' at line $line_num (new definition: '$mnemonic', previous definition: '$prev_defs')");
    } else {
      @mnem_array = @{$hex2mnem{lc $op_hex_str}}; # Recover the existing array to add more entries.
      push @mnem_array, $mnemonic;
      $hex2mnem{lc $op_hex_str} = \@mnem_array;
    }
  }
  return;
} # load_hex2mnem_entry


#######################################################################
#
# Convert a decimal number to 4-digit hex string
#
sub dec2hex4 {
  my $dec = shift;
  my $hex_str = sprintf "%04X", $dec;
  return ( $hex_str );
} # dec2hex4

sub d2h { # Quickie version for use with Perl debugger.
  return dec2hex4(@_);
} # d2h


#######################################################################
#
# Convert a decimal number to 2-digit hex string
#
sub dec2hex2 {
  my $dec = shift;
  my $hex_str = sprintf "%02X", $dec;
  return ( $hex_str );
} # dec2hex2

sub d2h2 { # Quickie version for use with Perl debugger.
  return dec2hex2(@_);
} # d2h2

#######################################################################
#
# Convert a hex string to a decimal number
#
sub hex2dec {
  my $hex_str = shift;
  my $dec = hex("0x" . $hex_str);
  return ( $dec );
} # hex2dec

sub h2d { # Quickie version for use with Perl debugger.
  return hex2dec(@_);
} # h2d


#######################################################################
#
# Read in a file and redact any commented sections, if requested to do so,
#
sub read_file {
  my $file = shift;
  my $do_redact = shift;
  local $_;
  my (@lines);
  open FILE, $file or die_msg(this_function((caller(0))[3]), "Cannot open input file '$file' for reading: $!");
  while( <FILE> ) {
    s/\r//; s/\n//;
    push @lines, $_;
  }
  close FILE;

  # We need to enforce the END statement being the last line in a V3 source. So
  # redact the comments when in V3 mode so we can more easily recognize when
  # the last statement is not an 'END'.
  if ($do_redact or $v3_mode) {
    @lines = redact_comments(@lines);
    @lines = remove_blank_lines(@lines);
  }

  @lines = remove_line_numbers(@lines);

  # Enforce the last line being an END if in V3 mode, unless we are XROM mode.
  if ($v3_mode and not $xrom_mode) {
    # Make sure there is an END as the last instruction. If not, add one.
    if (not @lines) {
      push @lines, "END";
      print "// WARNING: $script_name: V3 mode: Empty program in '$file'. Adding \"END\" as last statement in source...\n";
    } elsif ($lines[-1] !~ /(^|\s+)END($|\s+)/) {
      push @lines, "END";
      print "// WARNING: $script_name: V3 mode: Missing terminal \"END\" in file '$file'. Appending after last statement in source...\n";
    }
  }

  return @lines;
} # read_file


#######################################################################
#
# Redact any commented sections.
#
# Currently this will handle "//", "/* ... */" on a single line, "/*" alone
# on a line up to and including the next occurrence of "*/", plus more than one
# occurrence of "/* ... */ ... /* ... */" on a single line. There may be some
# weird cases that it does not handle.
#
sub redact_comments {
  my @lines = @_;
  local $_;

  # Oh no!! Slanted toothpick syndrome! :-)
  my $slc  = "\/\/"; # Single line comment marker.
  my $mlcs = "/\\*"; # Multiline comment start marker.
  my $mlce = "\\*/"; # Multiline comment end marker.
  my $in_mlc = 0;    # In-multiline-comment flag.
  my (@redacted_lines);

  foreach (@lines) {
    s/\r//; s/\n//;

    # Detect if we are currently in a multiline comment and we see the end of a
    # multiline comment. If so, remove up to and including the trailing multiline
    # comment marker.
    if( $in_mlc and m<${mlce}> ) {
      $in_mlc = 0;
      s<^.*?\*/><>;
    }

    # Remove any single line comment sections from the line.
    s<${slc}.*$><>;

    # Detect complete multiline comments ...and remove them. (don't be greedy)
    while( m<${mlcs}.+?${mlce}> ) {
      s<${mlcs}.*?${mlce}><>;
    }

    # If we still have a start of a multiline comment, set the flag on (possibly again).
    # It means we don't have a terminated comment set so delete to the end of the line.
    if( m<${mlcs}> ) {
      $in_mlc = 1;
      s<${mlcs}.*$><>;
    }

    # If we are still in a multiline comment and we get here, scrub the entire line.
    if( $in_mlc ) {
      $_ = "";
    }

    # Store anything that is left.
    push @redacted_lines, $_;
  }
  return @redacted_lines;
} # redact_comments


#######################################################################
#
# Remove any array elements that are blank lines.
#
sub remove_blank_lines {
  my @lines = @_;
  my (@new_lines);
  local $_;
  foreach (@lines) {
    next if /^\s*$/;
    push @new_lines, $_;
  }
  return @new_lines;
} # remove_blank_lines


#######################################################################
#
# Remove any leading line numbers.
#
sub remove_line_numbers {
  my @lines = @_;
  local $_;
  foreach (@lines) {
    s/^\s*\d{3,4}\:{0,1}//;
  }
  return @lines;
} # remove_line_numbers




#######################################################################
#
# Evaluate a possible hex value from a string.
#
sub eval_possible_hex {
  my $num = shift; # Either a decimal value or a hex string (the latter with or without the leading "0x").
  my $result = 0;
  if( $num =~ /^\s*0x/ ) {
    $result = hex($num);
  } elsif( $num =~ /[a-f-A-F]+/ ) {
    $result = hex("0x" . $num);
  } else {
    $result = $num;
  }
  return $result; # A decimal value
} # eval_possible_hex


#######################################################################
#
# Build an empty flash image.
#
sub build_empty_flash_image {
  my $outfile = shift;
  my ($crc16);
  my @words = pre_fill_flash($flash_blank_fill_hex_str); # A hex string.
  my $next_free_word = 1;
  $words[1] = $next_free_word;
  $crc16 = ($use_magic_marker) ? $MAGIC_MARKER : calc_crc16( @words[2 .. $next_free_word] );
  $words[0] = $crc16;
  write_binary( $outfile, @words );
  print "// CRC16: ", dec2hex4($crc16), "\n";
  print "// Total words: ", $next_free_word-1, "\n";
  return;
} # build_empty_flash_image


#######################################################################
#
# Run the preprocessor on the array of lines.
#
sub run_pp {
  my $ref_src_lines = shift; # Single concatenated array of all source files.
  my @files = @_;
  my (@lines, $err_msg, $cmd, $pp_location);
  local $_;

  # Create a temporary intermediate file holding the raw sources concatenated together.
  my $tmp_file = gen_random_writeable_filename();
  open TMP, "> $tmp_file" or die_msg(this_function((caller(0))[3]), "Cannot open temp file '$tmp_file' for writing: $!");
  foreach (@{$ref_src_lines}) {
    print TMP "$_\n";
  }
  close TMP;

  # See if we can locate the preprocessor.
  # Order of precedence is:
  # 1) Specified dir from command line.
  # 2) local direct
  # 3) where the main script is running from
  debug_msg(this_function((caller(0))[3]), "Base name of the preprocessor being searched for: '${preproc}'") if $debug;
  if( $preproc_fallback_dir and -e "${preproc_fallback_dir}${preproc}" ) {
    $pp_location = "${preproc_fallback_dir}${preproc}";
  } elsif( -e "${preproc}" ) {
    $pp_location = "${preproc}";
  } elsif( -e "${script_dir}${preproc}" ) {
    $pp_location = "${script_dir}${preproc}";
  } else {
    my $locations = "'${preproc}' or '${script_dir}${preproc}'";
    if( $preproc_fallback_dir ) {
      $locations .= " or '${preproc_fallback_dir}${preproc}'";
    }
    die_msg(this_function((caller(0))[3]), "Cannot locate the assembly preprocessor in $locations");
  }
  $pp_location =~ s:\\:/:g;
  $pp_options = $no_skip_override if $no_skip_override;
  $cmd = "${pp_location} $pp_options -cat $tmp_file";

  if ($v3_mode) {
    $cmd .= " -v3";
  }

  if ($v3_mode and $xrom_mode) {
    $cmd .= " -xrom";
  }

  # The new spawned job linkage mechanism sends all daughter output to STDOUT, even what used
  # to be sent to STDERR. THIS SHOULD ONLY HAPPEN WHEN A JOB IS SPAWNED. (This is to "fix"
  # deficiencies in the MS-DOS COMMAND.EXE shell.) When stand-alone, errors are still sent to
  # STDERR. What is lost is the fact that debug statements are no longer sent to STDERR where
  # they should have been but contaminate the output stream. (SIGH!!) This makes debugging MUCH
  # more challenging.
  $cmd .= " -e2so";

  # Override the step digits as required.
  # XXX Hah! Who knew?! In Perl, 'log' is actually log2, not log10, like everywhere else!!
  my $sd = ceil(log($max_flash_words)/log(10));

  $cmd .= " -sd $sd";

  # Force the colour mode to be off if in debug mode because with this damn MS-DOS STDERR
  # thing, all the colour attributes contaminate the STDOUT stream and the ASM cannot parse
  # them correctly. (It could help if I put an ANSI filter in the reader.)
  if ($debug) {
    $cmd .= " -colour_mode 0";
    $cmd .= " -d $debug";
  } else {
    $cmd .= " -colour_mode $use_ansi_colour";
  }
  $cmd .= " $pp_args";

  print "// Running WP 34S preprocessor from: $pp_location\n";
  debug_msg(this_function((caller(0))[3]), "Running: '$cmd'")
    if ($debug > 2) or (exists $ENV{WP34S_ASM} and ($ENV{WP34S_ASM} =~ /PRESERVE_PP_DBG/i));

  @lines = `$cmd`;
  $err_msg = $?;
  if( $err_msg ) {
    warn_msg(this_function((caller(0))[3]), "WP 34S preprocessor failed. Temp file: '$tmp_file'");
    die_msg(this_function((caller(0))[3]), "Perhaps you can try running it in isolation using: \$ $cmd");
  }
  unlink $tmp_file unless exists $ENV{WP34S_ASM} and ($ENV{WP34S_ASM} =~ /PRESERVE_PP_TMP/i);

  my (@clean_lines);
  foreach (@lines) {
    s/\r//; s/\n//;
    push @clean_lines, $_;
  }

  open PP_LST, "> $pp_listing" or die_msg(this_function((caller(0))[3]), "Cannot open preprocessor list file '$pp_listing' for writing: $!");
  print PP_LST "// $script_name: Source file(s): ", join (", ", @files), "\n";
  foreach (@clean_lines) {
    print PP_LST "$_\n";
  }
  close PP_LST;

  return @clean_lines;
} # run_pp


#######################################################################
#
# Dump the output into a C-array.
#
sub dump_c_array {
  my $file = shift;
  my $leader = shift;
  my $indent_spaces = shift;
  my @val_array = @_;
  open OUT, "> $file" or die_msg(this_function((caller(0))[3]), "Could not open C-array file '$file' for writing: $!");
  print OUT "$leader\n";
  for my $hex_str (0 .. (scalar @val_array)-2) {
    printf OUT "%0s0x%04x,\n", " " x $indent_spaces, $val_array[$hex_str];
  }
  printf OUT "%0s0x%04x };\nconst unsigned short int xrom_size = sizeof(xrom) / sizeof(const s_opcode);\n", " " x $indent_spaces, $val_array[-1];
  close OUT;
  return;
} # dump_c_array


#######################################################################
#
# Generate a random file name that can be used and thrown away.
# It is thrown away by the consumer, not in here!
#
sub gen_random_writeable_filename {
  my $filename = rand();
  $filename = ".__${filename}.tmp";
  my $attempts_left = 10;
  while( -e "$filename" ) {
    $filename = rand();
    $filename = ".__${filename}.tmp";
    $attempts_left--;
    if( $attempts_left < 0 ) {
      die_msg(this_function((caller(0))[3]), "Could not succeed in creating a temporary file: $!");
    }
  }
  return $filename;
} # gen_random_writeable_filename


#######################################################################
#
# Zero extend the operand. For example, modify the following:
#
#   RCL 5
#
# to:
#
#   RCL 05
#
sub zero_extend_operand {
  my $operand = shift;
  $operand =~ s/(\s+)(\d+)/${1}0${2}/; # Retain the exact whitespace!
  return $operand;
} # zero_extend_operand


#######################################################################
#
# Replace a string without using RegExp.
# See: http://www.bin-co.com/perl/scripts/str_replace.php
#
sub str_replace {
  my ($replace_this, $with_this, $string, $do_globally) = @_;

  my $len = length($replace_this);
  my $position = 0; # current position

  while( ($position = index($string, $replace_this, $position)) >= 0 ) {
    substr($string, $position, $len) = $with_this;
    if( not defined $do_globally or not $do_globally ) {
      last;
    }
  }
  return $string;
} # str_replace


#######################################################################
#
# Convert 2-digit BACK/SKIP to 3-digit versions.
# Far better to use PP's JMP instruction in the first place!
#
sub conv_skip_back_v2_to_v3 {
  my @src = @_;
  for (my $k = 0; $k < scalar @src; $k++) {
    if ($src[$k] =~ /(SKIP|BACK)\s+(\d{2})($|\s+)/) {
      my $offset2 = $2;
      my $offset3 = sprintf "%03d", $offset2;
      # Replace the 2-digit SKIP/BACK with a 3-digit variant.
      # Be careful to put the white space back exactly as it was found!
      $src[$k] =~ s/(SKIP|BACK)(\s+)${offset2}/${1}${2}${offset3}/;
    }
  }
  return @src;
} # conv_skip_back_v2_to_v3


#######################################################################
#
#
#
sub extract_svn_version {
  my $svn_rev = "";
  if( $SVN_Current_Revision =~ /Rev.+?(\d+)\s*\$/ ) {
    $svn_rev = $1;
  }
  return $svn_rev;
} # extract_svn_version


#######################################################################
#
# Format a fatal message.
#
sub die_msg {
  my $func_name = shift;
  my $text = shift;
  my $msg = "";

  $msg = "$ansi_red_bg" if $use_ansi_colour;
  $msg .= "ERROR: $func_name:";
  $msg .= "$ansi_normal " if $use_ansi_colour;
  $msg .= " $text";
  if ($stderr2stdout) {
    print "$msg\n";
    die "\n";
  } else {
    die "$msg\n";
  }
} # die_msg


#######################################################################
#
# Format a warning message.
#
sub warn_msg {
  my $func_name = shift;
  my $text = shift;
  my $msg = "";

  $msg = "$ansi_rev_red_bg" if $use_ansi_colour;
  $msg .= "WARNING: $func_name:";
  $msg .= "$ansi_normal " if $use_ansi_colour;
  $msg .= " $text";
  if ($stderr2stdout) {
    print "$msg\n";
  } else {
    warn "$msg\n";
  }
} # warn_msg


#######################################################################
#
# Format a debug message.
#
sub debug_msg {
  my $func_name = shift;
  my $text = shift;
  my $msg = "";

  $msg = "$ansi_green_bg" if $use_ansi_colour;
  $msg .= "// DEBUG: $func_name:";
  $msg .= "$ansi_normal " if $use_ansi_colour;
  $msg .= " $text";
  print "$msg\n";
} # debug_msg


#######################################################################
#
# Swap the main:: for the actual script name.
#
sub this_function {
  my $this_function = shift;
  $this_function = "main" if not defined $this_function or ($this_function eq "");
  $this_function =~ s/main/$script_name/;
  return $this_function;
} # this_function


########################################################################
#######################################################################
#
# Process the command line option list.
#
sub get_options {
  my ($arg);
  while ($arg = shift(@ARGV)) {

    # See if help is asked for
    if( $arg eq "-h" ) {
      print "$usage\n";
      die "\n";
    }

    if( $arg eq "-more_help" ) {
      print "$usage\n";
      print "$extended_help\n";
      die "\n";
    }

    elsif( ($arg eq "--version") or ($arg eq "-V") ) {
      print "$script\n";
      my $svn_rev = extract_svn_version();
      print "SVN version: $svn_rev\n" if $svn_rev;
      die "\n";
    }

    elsif( ($arg eq "-opcodes") or ($arg eq "-opcode") or ($arg eq "-map") or  ($arg eq "-op") ) {
      $opcode_map_file = shift(@ARGV);
    }

    elsif( $arg eq "-dis" ) {
      $mode = $DISASSEMBLE_MODE;
    }

    elsif( $arg eq "-d" ) {
      $debug = shift(@ARGV);
    }

    elsif( $arg eq "-v" ) {
      $quiet = 0;
    }

    elsif( $arg eq "-svn" ) {
      $report_op_svn = 1;
    }

    elsif( $arg eq "-no_svn" ) {
      $report_op_svn = 0;
    }

    elsif( ($arg eq "-s") or ($arg eq "-stars") ) {
      $star_labels = shift(@ARGV);
    }

    elsif( ($arg eq "-l") or ($arg eq "-lib") or ($arg eq "-flash") ) {
      $flash_mode = 1;
    }

    elsif( $arg eq "-fill" ) {
      $user_flash_blank_fill = dec2hex4(eval_possible_hex(shift(@ARGV)));
    }

    elsif( $arg eq "-dl" ) {
      $disable_flash_limit = 1;
    }

    elsif( $arg eq "-ns" ) {
      $no_step_numbers = 1;
    }

    elsif( $arg eq "-o" ) {
      $outfile = shift(@ARGV);
    }

    elsif( ($arg eq "-e2so") or ($arg eq "-stderr2stdout")) {
      $stderr2stdout = 1;
    }

    elsif( $arg eq "-pp" ) {
      $enable_pp = 1;
    }

    elsif( $arg eq "-pp_script" ) {
      $preproc = shift(@ARGV);
    }

    elsif( $arg eq "-pp_dir" ) {
      $preproc_fallback_dir = shift(@ARGV);
      unless( $preproc_fallback_dir =~ /\/$/ ) {
        $preproc_fallback_dir .= "/";
      }
    }

    elsif ($arg eq "-pp_args") {
      $pp_args = shift(@ARGV);
    }

    elsif ($arg eq "-noskip") {
      $no_skip_override = " -m 1 ";
    }

    elsif( $arg eq "-nc" ) {
      $use_ansi_colour = 0;
    }

    # Undocumented debug hook to zero the last 4 words.
    elsif( $arg eq "-04" ) {
      $zero_last_4 = 1;
    }

    # Undocumented debug hook to generate a "universal" CRC.
    elsif( $arg eq "-magic" ) {
      $use_magic_marker = 1;
    }

    # Undocumented debug hook to generate an empty flash image.
    elsif( $arg eq "-empty" ) {
      $build_an_empty_image = 1;
    }

    # Undocumented debug hook to generate a XROM C-array.
    # Automatically disables the flash length limit since the XROM can be larger than a
    # flash image.
    elsif( ($arg eq "-c_xrom") or ($arg eq "-c") ) {
      $xrom_mode = $XROM_C_MODE;
      $disable_flash_limit = 1;
    }

    # Undocumented debug hook to generate a XROM binary image.
    # Automatically disables the flash length limit since the XROM can be larger than a
    # flash image.
    elsif( $arg eq "-xrom" ) {
      $xrom_mode = $XROM_BIN_MODE;
      $disable_flash_limit = 1;
    }

    # This is typically only used for debug at the moment. It will dump the expanded tables
    # into a file of this name. Just having the file named is sufficient to trigger this mode.
    elsif( ($arg eq "-e") or ($arg eq "-expand") or ($arg eq "-syntax") ) {
      $expanded_op_file = shift(@ARGV);
    }

    # This is typically only used for debug at the moment. It will dump the escaped alpha table
    # into a file of this name. Just having the file named is sufficient to trigger this mode.
    elsif( ($arg eq "-da") or ($arg eq "-alpha") ) {
      $dump_escaped_alpha_table = shift(@ARGV);
    }

    elsif( $arg eq "-nc" ) {
      $use_ansi_colour = 0;
    }

    elsif( ($arg eq "-ac") or ($arg eq "-colour") or ($arg eq "-color") ) {
      $use_ansi_colour = 1;
    }

    elsif ($arg eq "-colour_mode") {
      $use_ansi_colour = shift(@ARGV);
    }

    elsif ($arg eq "-sb2to3") {
      $conv_skip_back_2to3 = 1;
    }

    elsif (($arg eq "-dis_alias") or ($arg eq "-ds")) {
      $choose_disassemble_opcode = -1;
    }

    elsif (($arg eq "-no_dis_alias") or ($arg eq "-nda")) {
      $choose_disassemble_opcode = 0;
    }

    else {
      push @files, $arg;
    }
  }

  #----------------------------------------------
  # Verify that we have an output file if we are in assembler mode.
  if( ($mode eq $ASSEMBLE_MODE) and ($outfile eq "-") and not $expanded_op_file ) {
    die_msg(this_function((caller(0))[3]), "Must enter an output file name in assembler mode (-o SomeFileName).\n Enter '$script_name -h' for help.");
  }

  if( $enable_pp ) {
    if( $mode ne $DISASSEMBLE_MODE ) {
      print "// WP 34S assembly preprocessor enabled: '-pp'\n";
    } else {
      print "// NOTE: The preprocessor switch (-pp) has no effect in disassembly mode.\n";
    }
  }

  return;
} # get_options
