#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <algorithm>
#include <atomic>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include "Knob_arcs.hpp"



// ============================================================================
// ArpLib  (22 HP)
// ============================================================================
//
// OVERVIEW
// --------
// ArpLib is a chord-progression arpeggiator. It stores 20 user-editable chord
// sequences ("slots"), each backed by one of 20 style groups (10 minor, 10
// major) with 50 progressions apiece, and plays an arpeggio over the tones
// of the currently active slot. Every chord can be individually overridden
// per slot from the on-screen chord strip, which opens a circle-of-fifths
// drop-down with fourteen chord qualities per root. A RNDNEXT switch can turn
// the fixed-order chord advance into a random walk through the progression.
//
// CHORD PROGRESSION LIBRARY
// -------------------------
// The library is built from a handful of "base" 50-entry lists (MIN_DARK,
// MIN_GOTHIC, MIN_PSY, MIN_CLASSICAL, MIN_JAZZ, MIN_EXP, MAJ_HAPPY,
// MAJ_CINEMA, MAJ_JAZZ, MAJ_CLASSICAL). Additional "family" groups are derived
// from a base list by a token-for-token chord substitution (e.g. every "G"
// becomes "G7"), so derivative groups share the rhythmic structure of their
// base list while sounding in a different harmonic colour. All base lists are
// written in the key of C minor / C major; the display transposes them to the
// current ROOT setting.
//
// PATTERNS (PATTERN knob, snapped)
// --------------------------------
//   0 UP          rise through the current chord's note pool
//   1 DOWN        fall through the pool
//   2 UP-DOWN     rise then fall (no repeat of the endpoints)
//   3 DOWN-UP     fall then rise (no repeat of the endpoints)
//   4 CONVERGE    alternate lowest / highest inward
//   5 DIVERGE     alternate highest / lowest outward
//   6 RANDOM      uniformly pick a pool index each step
//   7 WALK        biased random walk, one step up or down at a time
//   8 THUMB       root on even steps, other notes on odd steps
//
// CHORD SEQUENCES (SEQ knob + SEQ CV, 1..20)
// ------------------------------------------
// Each of the 20 stored slots remembers: which style group it uses, which
// progression (#1..#50) within that group, and any per-chord user overrides
// (the "custom" array). The factory contents put progression #k of the
// "Dark / Melancholic" group into slot k.
//
// Two-way SEQ control:
//   * With nothing patched, the SEQ knob directly selects the active slot.
//   * With SEQ CV patched, the CV (0..10 V) selects one of the 20 slots and
//     the SEQ knob follows the CV. Turning the knob by hand still works and
//     wins until the CV moves to a different slot.
//
// RNDNEXT (FEEL row, rightmost cell)
// ----------------------------------
// When enabled, chord advances pick the next chord uniformly at random from
// the progression instead of stepping through it in order. `rndNextRepeat`
// (context-menu option) decides whether the same chord may be picked twice
// in a row. The LOOP output fires every `numChords` random chord changes,
// matching the wrap interval of the ordered mode.
//
// OUTPUTS
// -------
//   PITCH    arpeggio note (1 V/oct)
//   GATE     gate for the current note
//   TRIG     short pulse on every step
//   VEL      velocity of the last note (0..10 V)
//   CHORD    polyphonic: all notes of the current chord
//   CHG      trigger on every chord change
//   LOOP     trigger when the progression wraps (ordered) or after `n`
//            random chord changes (RNDNEXT)
//
// INPUTS
// ------
//   CLOCK    rising-edge clock; period is measured for gate/swing timing
//   RESET    resets chord, step, swing counters and the RNDNEXT loop counter
//   ROOT CV  1 V/oct root transposition (quantized to semitones)
//   RND CV   adds to the RANDOM knob (0..10 V -> 0..100%)
//   PROG CV  0..10 V selects progression #1..#50 within the active group
//   SEQ CV   0..10 V = slots 1..20; the SEQ knob follows the CV
//
// CONTROLS
// --------
//   NOTES row: ROOT, OCT, RANGE, PATTERN, STEPS, SEQ, RESET
//   FEEL row:  RANDOM (large), PROB, GATE, SWING, HUMAN, RNDNEXT switch
//
// File structure:
//   1. Chord progression library (namespace arplib).
//   2. Chord parser (namespace arplib).
//   3. Circle-of-fifths / chord-quality tables for the drop-down.
//   4. ArpLib module (state, persistence, audio processing).
//   5. Panel layout constants (namespace lay).
//   6. Static backdrop (flat AGMM panel).
//   7. Live display (chord strip + step history piano roll).
//   8. Knob arcs overlay.
//   9. ArpLibWidget and model registration.
// ============================================================================


// ============================================================================
//  CHORD PROGRESSION LIBRARY
//  Base lists are typed in; the "family" groups are derived from them by a
//  token-for-token chord substitution (exactly how the original lists relate).
// ============================================================================
namespace arplib
{

	// -- Base minor progressions (all in C minor). 50 entries each. ---------
	static const char *const MIN_DARK[50] = {
		"Cm Ab Eb Bb", "Cm Bb Ab G", "Cm Fm Ab G", "Cm Eb Ab G", "Cm Fm Gm Cm",
		"Cm Ab Fm G", "Cm Bb Gm Ab", "Cm Eb Bb Ab", "Cm Ab Bb G", "Cm Fm Bb G",
		"Cm Gm Ab Fm", "Cm Eb Fm G", "Cm Bb Eb G", "Cm Ab Eb G", "Cm Fm Ab Bb",
		"Cm Gm Fm G", "Cm Ab G Cm", "Cm Eb Fm Cm", "Cm Bb Fm G", "Cm Fm Eb Bb",
		"Cm Ab Bb Cm", "Cm Eb G Cm", "Cm Fm G Ab", "Cm Bb Ab Eb", "Cm Gm Bb Ab",
		"Cm Ab Fm Cm", "Cm Eb Bb G", "Cm Fm Bb Cm", "Cm Bb G Ab", "Cm Ab Eb Fm",
		"Cm Fm Cm G", "Cm Eb Ab Fm", "Cm Bb Eb Fm", "Cm G Ab Bb", "Cm Ab G Eb",
		"Cm Fm G Eb", "Cm Eb Bb Cm", "Cm Ab Fm Bb", "Cm Gm Eb Ab", "Cm Bb Fm Eb",
		"Cm Fm Ab Cm", "Cm Eb G Bb", "Cm Ab Bb Eb", "Cm Gm G Cm", "Cm Bb Ab Fm",
		"Cm Eb Fm Ab", "Cm Ab Eb Cm", "Cm Fm Bb Eb", "Cm G Fm Cm", "Cm Bb Eb Ab"};

	static const char *const MIN_GOTHIC[50] = {
		"Cm Ab Db G", "Cm G Ab Ddim", "Cm Bdim Ab Ddim", "Cm Db Ab Ddim", "Cm Bdim Fm Cm",
		"Cm Ab Bdim Ddim", "Cm G Fm Ab", "Cm Db G Ab", "Cm Ab G Ddim", "Cm Bdim G Ddim",
		"Cm Fm Ab Bdim", "Cm Db Bdim Ddim", "Cm G Db Ddim", "Cm Ab Db Ddim", "Cm Bdim Ab G",
		"Cm Fm Bdim Ddim", "Cm Ab Ddim Cm", "Cm Db Bdim Cm", "Cm G Bdim Ddim", "Cm Bdim Db G",
		"Cm Ab G Cm", "Cm Db Ddim Cm", "Cm Bdim Ddim Ab", "Cm G Ab Db", "Cm Fm G Ab",
		"Cm Ab Bdim Cm", "Cm Db G Ddim", "Cm Bdim G Cm", "Cm G Ddim Ab", "Cm Ab Db Bdim",
		"Cm Bdim Cm Ddim", "Cm Db Ab Bdim", "Cm G Db Bdim", "Cm Ddim Ab G", "Cm Ab Ddim Db",
		"Cm Bdim Ddim Db", "Cm Db G Cm", "Cm Ab Bdim G", "Cm Fm Db Ab", "Cm G Bdim Db",
		"Cm Bdim Ab Cm", "Cm Db Ddim G", "Cm Ab G Db", "Cm Fm Ddim Cm", "Cm G Ab Bdim",
		"Cm Db Bdim Ab", "Cm Ab Db Cm", "Cm Bdim G Db", "Cm Ddim Bdim Cm", "Cm G Db Ab"};

	static const char *const MIN_PSY[50] = {
		"Cm Ab Bb Gm", "Cm Gm Ab Eb", "Cm Fm Ab Eb", "Cm Bb Ab Eb", "Cm Fm G Cm",
		"Cm Ab Fm Eb", "Cm Gm G Ab", "Cm Bb Gm Ab", "Cm Ab Gm Eb", "Cm Fm Gm Eb",
		"Cm G Ab Fm", "Cm Bb Fm Eb", "Cm Gm Bb Eb", "Cm Ab Bb Eb", "Cm Fm Ab Gm",
		"Cm G Fm Eb", "Cm Ab Eb Cm", "Cm Bb Fm Cm", "Cm Gm Fm Eb", "Cm Fm Bb Gm",
		"Cm Ab Gm Cm", "Cm Bb Eb Cm", "Cm Fm Eb Ab", "Cm Gm Ab Bb", "Cm G Gm Ab",
		"Cm Ab Fm Cm", "Cm Bb Gm Eb", "Cm Fm Gm Cm", "Cm Gm Eb Ab", "Cm Ab Bb Fm",
		"Cm Fm Cm Eb", "Cm Bb Ab Fm", "Cm Gm Bb Fm", "Cm Eb Ab Gm", "Cm Ab Eb Bb",
		"Cm Fm Eb Bb", "Cm Bb Gm Cm", "Cm Ab Fm Gm", "Cm G Bb Ab", "Cm Gm Fm Bb",
		"Cm Fm Ab Cm", "Cm Bb Eb Gm", "Cm Ab Gm Bb", "Cm G Eb Cm", "Cm Gm Ab Fm",
		"Cm Bb Fm Ab", "Cm Ab Bb Cm", "Cm Fm Gm Bb", "Cm Eb Fm Cm", "Cm Gm Bb Ab"};

	static const char *const MIN_CLASSICAL[50] = {
		"Cm Ddim Eb Fm", "Cm Fm Ddim Ab", "Cm G Ddim Ab", "Cm Eb Ddim Ab", "Cm G Bb Cm",
		"Cm Ddim G Ab", "Cm Fm Bb Ddim", "Cm Eb Fm Ddim", "Cm Ddim Fm Ab", "Cm G Fm Ab",
		"Cm Bb Ddim G", "Cm Eb G Ab", "Cm Fm Eb Ab", "Cm Ddim Eb Ab", "Cm G Ddim Fm",
		"Cm Bb G Ab", "Cm Ddim Ab Cm", "Cm Eb G Cm", "Cm Fm G Ab", "Cm G Eb Fm",
		"Cm Ddim Fm Cm", "Cm Eb Ab Cm", "Cm G Ab Ddim", "Cm Fm Ddim Eb", "Cm Bb Fm Ddim",
		"Cm Ddim G Cm", "Cm Eb Fm Ab", "Cm G Fm Cm", "Cm Fm Ab Ddim", "Cm Ddim Eb G",
		"Cm G Cm Ab", "Cm Eb Ddim G", "Cm Fm Eb G", "Cm Ab Ddim Fm", "Cm Ddim Ab Eb",
		"Cm G Ab Eb", "Cm Eb Fm Cm", "Cm Ddim G Fm", "Cm Bb Eb Ddim", "Cm Fm G Eb",
		"Cm G Ddim Cm", "Cm Eb Ab Fm", "Cm Ddim Fm Eb", "Cm Bb Ab Cm", "Cm Fm Ddim G",
		"Cm Eb G Ddim", "Cm Ddim Eb Cm", "Cm G Fm Eb", "Cm Ab G Cm", "Cm Fm Eb Ddim"};

	static const char *const MIN_JAZZ[50] = {
		"Cm9 Dm7b5 Ebmaj7 Fm9", "Cm9 Fm9 Dm7b5 Abmaj7", "Cm9 G7alt Dm7b5 Abmaj7", "Cm9 Ebmaj7 Dm7b5 Abmaj7", "Cm9 G7alt Bb13 Cm9",
		"Cm9 Dm7b5 G7alt Abmaj7", "Cm9 Fm9 Bb13 Dm7b5", "Cm9 Ebmaj7 Fm9 Dm7b5", "Cm9 Dm7b5 Fm9 Abmaj7", "Cm9 G7alt Fm9 Abmaj7",
		"Cm9 Bb13 Dm7b5 G7alt", "Cm9 Ebmaj7 G7alt Abmaj7", "Cm9 Fm9 Ebmaj7 Abmaj7", "Cm9 Dm7b5 Ebmaj7 Abmaj7", "Cm9 G7alt Dm7b5 Fm9",
		"Cm9 Bb13 G7alt Abmaj7", "Cm9 Dm7b5 Abmaj7 Cm9", "Cm9 Ebmaj7 G7alt Cm9", "Cm9 Fm9 G7alt Abmaj7", "Cm9 G7alt Ebmaj7 Fm9",
		"Cm9 Dm7b5 Fm9 Cm9", "Cm9 Ebmaj7 Abmaj7 Cm9", "Cm9 G7alt Abmaj7 Dm7b5", "Cm9 Fm9 Dm7b5 Ebmaj7", "Cm9 Bb13 Fm9 Dm7b5",
		"Cm9 Dm7b5 G7alt Cm9", "Cm9 Ebmaj7 Fm9 Abmaj7", "Cm9 G7alt Fm9 Cm9", "Cm9 Fm9 Abmaj7 Dm7b5", "Cm9 Dm7b5 Ebmaj7 G7alt",
		"Cm9 G7alt Cm9 Abmaj7", "Cm9 Ebmaj7 Dm7b5 G7alt", "Cm9 Fm9 Ebmaj7 G7alt", "Cm9 Abmaj7 Dm7b5 Fm9", "Cm9 Dm7b5 Abmaj7 Ebmaj7",
		"Cm9 G7alt Abmaj7 Ebmaj7", "Cm9 Ebmaj7 Fm9 Cm9", "Cm9 Dm7b5 G7alt Fm9", "Cm9 Bb13 Ebmaj7 Dm7b5", "Cm9 Fm9 G7alt Ebmaj7",
		"Cm9 G7alt Dm7b5 Cm9", "Cm9 Ebmaj7 Abmaj7 Fm9", "Cm9 Dm7b5 Fm9 Ebmaj7", "Cm9 Bb13 Abmaj7 Cm9", "Cm9 Fm9 Dm7b5 G7alt",
		"Cm9 Ebmaj7 G7alt Dm7b5", "Cm9 Dm7b5 Ebmaj7 Cm9", "Cm9 G7alt Fm9 Ebmaj7", "Cm9 Abmaj7 G7alt Cm9", "Cm9 Fm9 Ebmaj7 Dm7b5"};

	static const char *const MIN_EXP[50] = {
		"Cm Dbmaj7 E F#dim", "Cm F#dim Dbmaj7 Bbm", "Cm Ab Dbmaj7 Bbm", "Cm E Dbmaj7 Bbm", "Cm Ab G7 Cm",
		"Cm Dbmaj7 Ab Bbm", "Cm F#dim G7 Dbmaj7", "Cm E F#dim Dbmaj7", "Cm Dbmaj7 F#dim Bbm", "Cm Ab F#dim Bbm",
		"Cm G7 Dbmaj7 Ab", "Cm E Ab Bbm", "Cm F#dim E Bbm", "Cm Dbmaj7 E Bbm", "Cm Ab Dbmaj7 F#dim",
		"Cm G7 Ab Bbm", "Cm Dbmaj7 Bbm Cm", "Cm E Ab Cm", "Cm F#dim Ab Bbm", "Cm Ab E F#dim",
		"Cm Dbmaj7 F#dim Cm", "Cm E Bbm Cm", "Cm Ab Bbm Dbmaj7", "Cm F#dim Dbmaj7 E", "Cm G7 F#dim Dbmaj7",
		"Cm Dbmaj7 Ab Cm", "Cm E F#dim Bbm", "Cm Ab F#dim Cm", "Cm F#dim Bbm Dbmaj7", "Cm Dbmaj7 E Ab",
		"Cm Ab Cm Bbm", "Cm E Dbmaj7 Ab", "Cm F#dim E Ab", "Cm Bbm Dbmaj7 F#dim", "Cm Dbmaj7 Bbm E",
		"Cm Ab Bbm E", "Cm E F#dim Cm", "Cm Dbmaj7 Ab F#dim", "Cm G7 E Dbmaj7", "Cm F#dim Ab E",
		"Cm Ab Dbmaj7 Cm", "Cm E Bbm F#dim", "Cm Dbmaj7 F#dim E", "Cm G7 Bbm Cm", "Cm F#dim Dbmaj7 Ab",
		"Cm E Ab Dbmaj7", "Cm Dbmaj7 E Cm", "Cm Ab F#dim E", "Cm Bbm Ab Cm", "Cm F#dim E Dbmaj7"};

	// -- Base major progressions (all in C major). 50 entries each. ---------
	static const char *const MAJ_HAPPY[50] = {
		"C G Am F", "C Am F G", "Am F C G", "C F G C", "C Em Am F",
		"C G F G", "C Dm G C", "Am Dm G C", "C F Am G", "C Em F G",
		"C Am Dm G", "F G Em Am", "C G Em Am", "C Dm F G", "Am F Dm G",
		"C F Dm G", "C G Am Em", "C Em F C", "Am G F C", "C F C G",
		"C Am Em F", "Dm G C Am", "C Gsus4 F C", "C Cadd9 Gsus4 F", "C Cadd9 Gsus4 C",
		"C Cadd9 Gsus4 C", "C G Cadd9 F", "Am Gsus4 F C", "C Dm Gsus4 F", "F Cadd9 C G",
		"C Em Dm G", "Am Em F G", "C F Gsus4 G", "C Cadd9 F G", "C G Gsus4 F",
		"Dm F C G", "C Am F Em", "Em Am Dm G", "C F Em Am", "Am F G Em",
		"C G Dm F", "F C G Am", "C Em Gsus4 F", "C Cadd9 F G", "C Cadd9 F G",
		"Am Cadd9 F C", "C G Cadd9 F", "Dm G Am F", "C F Am Em", "C Gsus4 Am F"};

	static const char *const MAJ_CINEMA[50] = {
		"C F G Am", "C G Am F", "G Am C F", "C Am F C", "C Dm G Am",
		"C F Am F", "C Em F C", "G Em F C", "C Am G F", "C Dm Am F",
		"C G Em F", "Am F Dm G", "C F Dm G", "C Em Am F", "G Am Em F",
		"C Am Em F", "C F G Dm", "C Dm Am C", "G F Am C", "C Am C F",
		"C G Dm Am", "Em F C G", "C Bb Am C", "C Ab Bb Am", "C Ab Bb C",
		"C Ab Bb C", "C F Ab Am", "G Bb Am C", "C Em Bb Am", "Am Ab C F",
		"C Dm Em F", "G Dm Am F", "C Am Bb F", "C Ab Am F", "C F Bb Am",
		"Em Am C F", "C G Am Dm", "Dm G Em F", "C Am Dm G", "G Am F Dm",
		"C F Em Am", "Am C F G", "C Dm Bb Am", "C Ab Am F", "C Ab Am F",
		"G Ab Am C", "C F Ab Am", "Em F G Am", "C Am G Dm", "C Bb G Am"};

	static const char *const MAJ_JAZZ[50] = {
		"Cmaj9 Dm9 Em7 Fmaj7#11", "Cmaj9 Em7 Fmaj7#11 Dm9", "Em7 Fmaj7#11 Cmaj9 Dm9", "Cmaj9 Fmaj7#11 Dm9 Cmaj9", "Cmaj9 Am9 Em7 Fmaj7#11",
		"Cmaj9 Dm9 Fmaj7#11 Dm9", "Cmaj9 G13 Dm9 Cmaj9", "Em7 G13 Dm9 Cmaj9", "Cmaj9 Fmaj7#11 Em7 Dm9", "Cmaj9 Am9 Fmaj7#11 Dm9",
		"Cmaj9 Em7 G13 Dm9", "Fmaj7#11 Dm9 Am9 Em7", "Cmaj9 Dm9 Am9 Em7", "Cmaj9 G13 Fmaj7#11 Dm9", "Em7 Fmaj7#11 G13 Dm9",
		"Cmaj9 Fmaj7#11 G13 Dm9", "Cmaj9 Dm9 Em7 Am9", "Cmaj9 Am9 Fmaj7#11 Cmaj9", "Em7 Dm9 Fmaj7#11 Cmaj9", "Cmaj9 Fmaj7#11 Cmaj9 Dm9",
		"Cmaj9 Em7 Am9 Fmaj7#11", "G13 Dm9 Cmaj9 Em7", "Cmaj9 Bm7b5 Fmaj7#11 Cmaj9", "Cmaj9 E7 Bm7b5 Fmaj7#11", "Cmaj9 E7 Bm7b5 Cmaj9",
		"Cmaj9 E7 Bm7b5 Cmaj9", "Cmaj9 Dm9 E7 Fmaj7#11", "Em7 Bm7b5 Fmaj7#11 Cmaj9", "Cmaj9 G13 Bm7b5 Fmaj7#11", "Fmaj7#11 E7 Cmaj9 Dm9",
		"Cmaj9 Am9 G13 Dm9", "Em7 Am9 Fmaj7#11 Dm9", "Cmaj9 Fmaj7#11 Bm7b5 Dm9", "Cmaj9 E7 Fmaj7#11 Dm9", "Cmaj9 Dm9 Bm7b5 Fmaj7#11",
		"G13 Fmaj7#11 Cmaj9 Dm9", "Cmaj9 Em7 Fmaj7#11 Am9", "Am9 Em7 G13 Dm9", "Cmaj9 Fmaj7#11 Am9 Em7", "Em7 Fmaj7#11 Dm9 Am9",
		"Cmaj9 Dm9 G13 Fmaj7#11", "Fmaj7#11 Cmaj9 Dm9 Em7", "Cmaj9 Am9 Bm7b5 Fmaj7#11", "Cmaj9 E7 Fmaj7#11 Dm9", "Cmaj9 E7 Fmaj7#11 Dm9",
		"Em7 E7 Fmaj7#11 Cmaj9", "Cmaj9 Dm9 E7 Fmaj7#11", "G13 Dm9 Em7 Fmaj7#11", "Cmaj9 Fmaj7#11 Em7 Am9", "Cmaj9 Bm7b5 Em7 Fmaj7#11"};

	static const char *const MAJ_CLASSICAL[50] = {
		"C Dm Em F", "C Em F Dm", "Em F C Dm", "C F Dm C", "C Am Em F",
		"C Dm F Dm", "C G Dm C", "Em G Dm C", "C F Em Dm", "C Am F Dm",
		"C Em G Dm", "F Dm Am Em", "C Dm Am Em", "C G F Dm", "Em F G Dm",
		"C F G Dm", "C Dm Em Am", "C Am F C", "Em Dm F C", "C F C Dm",
		"C Em Am F", "G Dm C Em", "C Bdim F C", "C G7 Bdim F", "C G7 Bdim C",
		"C G7 Bdim C", "C Dm G7 F", "Em Bdim F C", "C G Bdim F", "F G7 C Dm",
		"C Am G Dm", "Em Am F Dm", "C F Bdim Dm", "C G7 F Dm", "C Dm Bdim F",
		"G F C Dm", "C Em F Am", "Am Em G Dm", "C F Am Em", "Em F Dm Am",
		"C Dm G F", "F C Dm Em", "C Am Bdim F", "C G7 F Dm", "C G7 F Dm",
		"Em G7 F C", "C Dm G7 F", "G Dm Em F", "C F Em Am", "C Bdim Em F"};

	using SubMap = std::vector<std::pair<std::string, std::string>>;

	// Splits a space-separated progression into individual chord tokens.
	static std::vector<std::string> tokenize(const std::string &s)
	{
		std::vector<std::string> out;
		std::string cur;
		for (char ch : s)
		{
			if (ch == ' ')
			{
				if (!cur.empty())
					out.push_back(cur);
				cur.clear();
			}
			else
				cur += ch;
		}
		if (!cur.empty())
			out.push_back(cur);
		return out;
	}

	// Single-pass token substitution (so swaps like Dm7<->Em7 work correctly).
	// Each token is looked up in `map` once; a match is replaced and the loop
	// moves on, so mutual swaps don't ping-pong.
	static std::vector<std::string> derive(const std::vector<std::string> &base, const SubMap &map)
	{
		std::vector<std::string> out;
		for (const std::string &prog : base)
		{
			std::string res;
			for (const std::string &tok : tokenize(prog))
			{
				std::string t = tok;
				for (const auto &kv : map)
				{
					if (kv.first == tok)
					{
						t = kv.second;
						break;
					}
				}
				if (!res.empty())
					res += ' ';
				res += t;
			}
			out.push_back(res);
		}
		return out;
	}

	static std::vector<std::string> fromArray(const char *const *a)
	{
		return std::vector<std::string>(a, a + 50);
	}

	struct Group
	{
		std::string name;
		std::vector<std::string> progs;
	};

	// Index 0..9 = C minor groups, 10..19 = C major groups.
	// Base lists come from the static tables above; derivative groups are
	// produced by chord substitution so their rhythm matches their base.
	static std::vector<Group> buildGroups()
	{
		std::vector<Group> g(20);
		auto minDark = fromArray(MIN_DARK);
		auto majHappy = fromArray(MAJ_HAPPY);

		// ---- Minor ----
		g[0] = {"Dark / Melancholic", minDark};
		g[1] = {"Dark Ambient / Drone", derive(minDark, {{"Fm", "Fm9"}, {"Gm", "Cmadd9"}, {"G", "Gm7"}, {"Ab", "Abmaj7"}, {"Eb", "Ebmaj7"}, {"Bb", "Bbadd9"}})};
		g[2] = {"Cinematic / Epic", derive(minDark, {{"G", "G7"}})};
		g[3] = {"Sad / Emotional", derive(minDark, {{"Cm", "Cm7"}, {"Fm", "Fm7"}, {"Gm", "Gm7"}, {"Ab", "Abmaj7"}, {"Eb", "Ebmaj7"}, {"G", "G7"}})};
		g[4] = {"Gothic / Horror", fromArray(MIN_GOTHIC)};
		g[5] = {"Psychedelic / Psytrance", fromArray(MIN_PSY)};
		g[6] = {"Trip-Hop / Post-Rock", derive(minDark, {{"Cm", "Cm9"}, {"Fm", "Fm7"}, {"Gm", "G7"}, {"G", "Gm7"}, {"Ab", "Abmaj7"}, {"Bb", "Bbadd9"}})};
		g[7] = {"Classical / Harmonic Minor", fromArray(MIN_CLASSICAL)};
		g[8] = {"Jazz / Neo-Soul (min)", fromArray(MIN_JAZZ)};
		g[9] = {"Experimental / Unusual", fromArray(MIN_EXP)};

		// ---- Major ----
		auto majEmo = derive(majHappy, {{"C", "Cmaj7"}, {"Am", "Am7"}, {"F", "Fmaj7"}, {"Dm", "Dm7"}, {"Em", "Em7"}});
		auto majJazz = fromArray(MAJ_JAZZ);
		auto majClassic = fromArray(MAJ_CLASSICAL);
		g[10] = {"Happy / Uplifting", majHappy};
		g[11] = {"Emotional / Nostalgic", majEmo};
		g[12] = {"Cinematic / Epic (maj)", fromArray(MAJ_CINEMA)};
		g[13] = {"Ambient / Dreamy", derive(majEmo, {{"Cmaj7", "Cmaj9"}, {"Am7", "Am9"}, {"Dm7", "Dm9"}, {"G", "G6"}, {"Cadd9", "Fadd9"}})};
		g[14] = {"Pop / Rock", derive(majHappy, {{"Gsus4", "Bb"}, {"Cadd9", "F/C"}})};
		g[15] = {"Jazz / Neo-Soul (maj)", majJazz};
		g[16] = {"Funk / Soul / R&B", derive(majJazz, {{"Cmaj9", "C7"}, {"Dm9", "Dm7"}, {"Am9", "Am7"}, {"Fmaj7#11", "F9"}, {"Bm7b5", "Bb9"}, {"E7", "D7"}})};
		g[17] = {"Classical / Traditional", majClassic};
		g[18] = {"Electronic / Synthwave", derive(majEmo, {{"Cmaj7", "Cadd9"}, {"G", "Gsus2"}, {"Gsus4", "Bb"}, {"Cadd9", "Ab"}, {"Em7", "Dm7"}, {"Dm7", "Em7"}})};
		g[19] = {"Experimental / Modal", derive(majClassic, {{"Em", "Eb"}, {"Am", "Ab"}, {"G7", "Am"}, {"Bdim", "Bb"}})};
		return g;
	}

	// Lazily-built singleton so the tables are only constructed once.
	static const std::vector<Group> &groups()
	{
		static const std::vector<Group> g = buildGroups();
		return g;
	}

	static const int NUM_MINOR_GROUPS = 10;
	static const int NUM_GROUPS = 20;

	// ------------------------------------------------------------------------
	//  Chord parsing
	// ------------------------------------------------------------------------
	// A chord is turned into up to MAX_TONES semitone offsets relative to C,
	// with roots 6..11 folded one octave down so the resulting pool always
	// crosses roughly the same pitch band. A slash bass note is prepended as
	// the lowest tone of the chord.
	static const int MAX_CHORDS = 8;
	static const int MAX_TONES = 8;

	struct ChordTones
	{
		int n = 0;
		int t[MAX_TONES] = {0};
	};

	struct IntervalDef
	{
		const char *name;
		int count;
		int iv[7];
	};

	// Recognised chord suffixes and their semitone offsets (relative to root).
	static const IntervalDef INTERVALS[] = {
		{"", 3, {0, 4, 7}},
		{"m", 3, {0, 3, 7}},
		{"dim", 3, {0, 3, 6}},
		{"7", 4, {0, 4, 7, 10}},
		{"maj7", 4, {0, 4, 7, 11}},
		{"m7", 4, {0, 3, 7, 10}},
		{"m9", 5, {0, 3, 7, 10, 14}},
		{"maj9", 5, {0, 4, 7, 11, 14}},
		{"add9", 4, {0, 4, 7, 14}},
		{"madd9", 4, {0, 3, 7, 14}},
		{"sus4", 3, {0, 5, 7}},
		{"sus2", 3, {0, 2, 7}},
		{"9", 5, {0, 4, 7, 10, 14}},
		{"13", 5, {0, 4, 10, 14, 21}},
		{"7alt", 5, {0, 4, 10, 15, 20}},
		{"m7b5", 4, {0, 3, 6, 10}},
		{"6", 4, {0, 4, 7, 9}},
		{"maj7#11", 5, {0, 4, 7, 11, 18}},
	};

	static int letterPC(char c)
	{
		switch (c)
		{
		case 'C':
			return 0;
		case 'D':
			return 2;
		case 'E':
			return 4;
		case 'F':
			return 5;
		case 'G':
			return 7;
		case 'A':
			return 9;
		case 'B':
			return 11;
		}
		return -1;
	}

	// Splits "Bbmaj7/D" -> rootPC=10, suffix="maj7", bassPC=2. Returns false on error.
	static bool splitChord(const std::string &s, int &rootPC, std::string &suffix, int &bassPC)
	{
		bassPC = -1;
		if (s.empty())
			return false;
		int pc = letterPC(s[0]);
		if (pc < 0)
			return false;
		size_t i = 1;
		if (i < s.size() && s[i] == '#')
		{
			pc++;
			i++;
		}
		else if (i < s.size() && s[i] == 'b')
		{
			pc--;
			i++;
		}
		rootPC = ((pc % 12) + 12) % 12;
		std::string rest = s.substr(i);
		size_t slash = rest.find('/');
		suffix = rest.substr(0, slash);
		if (slash != std::string::npos && slash + 1 < rest.size())
		{
			int b = letterPC(rest[slash + 1]);
			if (b >= 0)
			{
				if (slash + 2 < rest.size() && rest[slash + 2] == '#')
					b++;
				else if (slash + 2 < rest.size() && rest[slash + 2] == 'b')
					b--;
				bassPC = ((b % 12) + 12) % 12;
			}
		}
		return true;
	}

	// Tones are semitones relative to C (C = 0). Roots 6..11 are folded one octave down.
	static ChordTones parseChord(const std::string &s)
	{
		ChordTones c;
		int rootPC = 0, bassPC = -1;
		std::string suffix;
		if (!splitChord(s, rootPC, suffix, bassPC))
		{
			c.n = 3;
			c.t[0] = 0;
			c.t[1] = 4;
			c.t[2] = 7;
			return c;
		}
		const IntervalDef *def = &INTERVALS[0];
		for (const IntervalDef &d : INTERVALS)
		{
			if (suffix == d.name)
			{
				def = &d;
				break;
			}
		}
		int base = (rootPC >= 6) ? rootPC - 12 : rootPC;
		int lowest = 1000;
		for (int i = 0; i < def->count && c.n < MAX_TONES - 1; i++)
		{
			c.t[c.n] = base + def->iv[i];
			lowest = std::min(lowest, c.t[c.n]);
			c.n++;
		}
		if (bassPC >= 0)
		{
			int diff = (((lowest - bassPC) % 12) + 12) % 12;
			int bass = lowest - (diff == 0 ? 12 : diff);
			c.t[c.n++] = bass;
		}
		return c;
	}

	static const char *const FLAT_NAMES[12] = {"C", "Db", "D", "Eb", "E", "F", "Gb", "G", "Ab", "A", "Bb", "B"};

	// Display helper: transposes a chord name by `semis` semitones.
	// Slash chords carry their bass note along.
	static std::string transposeName(const std::string &s, int semis)
	{
		if (semis == 0)
			return s;
		int rootPC, bassPC;
		std::string suffix;
		if (!splitChord(s, rootPC, suffix, bassPC))
			return s;
		std::string out = FLAT_NAMES[(((rootPC + semis) % 12) + 12) % 12];
		out += suffix;
		if (bassPC >= 0)
		{
			out += "/";
			out += FLAT_NAMES[(((bassPC + semis) % 12) + 12) % 12];
		}
		return out;
	}

	// ------------------------------------------------------------------------
	//  Circle of fifths + chord qualities for the per-chord drop-down
	// ------------------------------------------------------------------------
	// Starting at C and moving by perfect fifths gives the classic ordering
	// C G D A E B F# Db Ab Eb Bb. Each entry is a pitch class 0..11.
	static const int CIRCLE_OF_FIFTHS[12] = {0, 7, 2, 9, 4, 11, 6, 1, 8, 3, 10, 5};

	struct QualityDef
	{
		const char *suffix;
		const char *desc;
	};

	// Chord qualities offered under each root in the drop-down.
	static const QualityDef QUALITIES[] = {
		{"", "major"},
		{"m", "minor"},
		{"dim", "diminished"},
		{"7", "dominant 7"},
		{"maj7", "major 7"},
		{"m7", "minor 7"},
		{"m7b5", "half-diminished"},
		{"6", "major 6"},
		{"add9", "add 9"},
		{"sus2", "suspended 2"},
		{"sus4", "suspended 4"},
		{"9", "dominant 9"},
		{"maj9", "major 9"},
		{"m9", "minor 9"},
	};
	static const int NUM_QUALITIES = (int)(sizeof(QUALITIES) / sizeof(QUALITIES[0]));

	static int mod12(int x)
	{
		return ((x % 12) + 12) % 12;
	}

	// Name used in the drop-down for a pitch class (6 is ambiguous).
	static std::string pcLabel(int pc)
	{
		return pc == 6 ? std::string("F#/Gb") : std::string(FLAT_NAMES[mod12(pc)]);
	}

	// True when two chord names describe the same root + quality (enharmonics included).
	static bool sameChord(const std::string &a, const std::string &b)
	{
		int ra, rb, ba, bb;
		std::string sa, sb;
		if (!splitChord(a, ra, sa, ba) || !splitChord(b, rb, sb, bb))
			return a == b;
		return ra == rb && sa == sb && ba == bb;
	}

} // namespace arplib


// ============================================================================
//  MODULE
// ============================================================================
struct ArpLib : Module
{
	// Parameter, input, output and light IDs must stay in declaration order.
	enum ParamId
	{
		ROOT_PARAM,
		OCTAVE_PARAM,
		RANGE_PARAM,
		PATTERN_PARAM,
		STEPS_PARAM,
		RANDOM_PARAM,
		GATE_PARAM,
		SWING_PARAM,
		PROB_PARAM,
		HUMAN_PARAM,
		RESET_PARAM,
		SEQ_PARAM,
		RNDNEXT_PARAM,
		PARAMS_LEN
	};
	enum InputId
	{
		CLOCK_INPUT,
		RESET_INPUT,
		ROOT_CV_INPUT,
		RANDOM_CV_INPUT,
		PROG_CV_INPUT,
		SEQ_CV_INPUT,
		INPUTS_LEN
	};
	enum OutputId
	{
		PITCH_OUTPUT,
		GATE_OUTPUT,
		TRIG_OUTPUT,
		VEL_OUTPUT,
		CHORD_OUTPUT,  // polyphonic: all notes of the current chord
		CHANGE_OUTPUT, // trigger on every chord change
		LOOP_OUTPUT,   // trigger when the progression restarts
		OUTPUTS_LEN
	};
	enum LightId
	{
		CLOCK_LIGHT,
		GATE_LIGHT,
		RESET_LIGHT,
		RNDNEXT_LIGHT,
		LIGHTS_LEN
	};

	enum Pattern
	{
		PAT_UP,
		PAT_DOWN,
		PAT_UPDOWN,
		PAT_DOWNUP,
		PAT_CONVERGE,
		PAT_DIVERGE,
		PAT_RANDOM,
		PAT_WALK,
		PAT_THUMB,
		PAT_LEN
	};

	static constexpr int POOL_MAX = 32;      // per-chord note pool
	static constexpr int ALLPOOL_MAX = 256;  // union of all chord pools
	static constexpr int NUM_STEP_OPTIONS = 10;
	const int stepOptions[NUM_STEP_OPTIONS] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};

	struct Pool
	{
		int n = 0;
		int note[ALLPOOL_MAX];
	};

	struct Progression
	{
		int n = 0;
		std::string names[arplib::MAX_CHORDS];
		arplib::ChordTones tones[arplib::MAX_CHORDS];
	};

	// ---- selection (persisted) ----
	int groupIdx = 0;
	int progIdx = 0;
	bool rndNextRepeat = false;		  // RNDNEXT may pick the same chord again
	bool randomFromChordOnly = false; // false: random notes come from the WHOLE progression

	// ---- progression hand-over between UI and audio thread ----
	// `staged` is written under mtx by the UI and consumed by the audio thread
	// the next time it wins the try-lock. `hasStaged` signals pending work.
	std::mutex mtx;
	Progression staged; // also used by the display (under mtx)
	std::atomic<bool> hasStaged{false};

	// ---- 20 stored chord sequences (all guarded by mtx) ----
	static constexpr int NUM_SEQ = 20;
	struct SeqSlot
	{
		int group = 0;
		int prog = 0;
		std::string custom[arplib::MAX_CHORDS]; // user-chosen chords (C-frame names, "" = preset)
	};
	SeqSlot slots[NUM_SEQ];
	int activeSlot = 0;
	int numChords = 4;
	arplib::ChordTones chords[arplib::MAX_CHORDS];
	Pool pools[arplib::MAX_CHORDS];
	Pool allPool;
	bool poolsDirty = true;
	int poolRange = 1;

	// ---- display info (read by widget) ----
	std::atomic<int> dispChord{0};
	std::atomic<int> dispTranspose{0};
	std::atomic<int> dispTick{0};		// steps played in the current chord
	std::atomic<int> dispSeq{1};		// active sequence slot, 1..20
	std::atomic<int> dispSteps{8};		// steps per chord
	std::atomic<float> dispRandom{0.f}; // effective randomness (knob + CV)

	// ---- sequencer state ----
	dsp::SchmittTrigger clockTrig, resetTrig;
	dsp::BooleanTrigger resetBtn;
	dsp::PulseGenerator trigPulse, changePulse, loopPulse;
	bool clockSeen = false;
	float timeSinceClock = 0.f;
	float period = 0.5f;
	uint64_t tickCounter = 0;
	int chordIdx = 0;
	int tickInChord = 1 << 30; // forces chord advance (-> chord 0) on first step
	int patternStep = 0;
	int walkIdx = 0;
	int chordsSinceLoop = 0; // RNDNEXT: chord changes since the last loop trigger
	bool pending = false;
	float pendingDelay = 0.f;
	float pendingStartDelay = 0.f;
	float gateRemain = 0.f;
	int currentNote = 0;
	float velocity = 8.f;
	int lastProgCv = -1;
	int lastCvSlot = -1;   // SEQ CV: slot the CV pointed at last time
	int lastKnobSlot = -1; // SEQ knob position last time (detects manual turns)

	// ---- step history for the display (written by audio thread, read by UI) ----
	struct StepRec
	{
		bool valid = false;
		bool rest = false;
		bool rnd = false;
		int note = 0;
		float len = 1.f; // gate length as a fraction of one clock tick
	};
	static constexpr int HIST_LEN = 16;
	StepRec hist[HIST_LEN];
	std::atomic<int> histHead{0};

	// Publish one step into the rolling history ring (audio thread only).
	void pushHist(int note, float len, bool rest, bool rnd)
	{
		int h = histHead.load();
		StepRec &r = hist[h];
		r.note = note;
		r.len = len;
		r.rest = rest;
		r.rnd = rnd;
		r.valid = true;
		histHead = (h + 1) % HIST_LEN;
	}

	ArpLib()
	{
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

		configSwitch(ROOT_PARAM, 0.f, 11.f, 0.f, "Root note",
					 {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"});
		configSwitch(OCTAVE_PARAM, -2.f, 2.f, 0.f, "Octave", {"-2", "-1", "0", "+1", "+2"});
		configSwitch(RANGE_PARAM, 1.f, 4.f, 1.f, "Octave range", {"1 oct", "2 oct", "3 oct", "4 oct"});
		configSwitch(PATTERN_PARAM, 0.f, PAT_LEN - 1, 0.f, "Pattern",
					 {"Up", "Down", "Up-Down", "Down-Up", "Converge", "Diverge", "Random", "Random walk", "Thumb (root bounce)"});
		configSwitch(STEPS_PARAM, 0.f, NUM_STEP_OPTIONS - 1, 5.f, "Clock steps per chord",
					 {"1", "2", "3", "4", "6", "8", "12", "16", "24", "32"});
		configParam(RANDOM_PARAM, 0.f, 1.f, 0.f, "Randomness", "%", 0.f, 100.f);
		configParam(GATE_PARAM, 0.05f, 1.f, 1.f, "Gate length", "%", 0.f, 100.f);
		configParam(SWING_PARAM, 0.f, 0.5f, 0.f, "Swing", "%", 0.f, 200.f);
		configParam(PROB_PARAM, 0.f, 1.f, 1.f, "Note probability", "%", 0.f, 100.f);
		configParam(HUMAN_PARAM, 0.f, 1.f, 0.f, "Velocity humanize", "%", 0.f, 100.f);
		configButton(RESET_PARAM, "Reset");
		configParam(SEQ_PARAM, 1.f, 20.f, 1.f, "Chord sequence (stored slot)");
		paramQuantities[SEQ_PARAM]->snapEnabled = true;
		configSwitch(RNDNEXT_PARAM, 0.f, 1.f, 0.f, "Random next chord",
					 {"Off (chords play in order)", "On (next chord chosen randomly)"});

		configInput(CLOCK_INPUT, "Clock");
		configInput(RESET_INPUT, "Reset");
		configInput(ROOT_CV_INPUT, "Root CV (1V/oct, quantized to semitones)");
		configInput(RANDOM_CV_INPUT, "Randomness CV (0-10V)");
		configInput(PROG_CV_INPUT, "Progression select CV (0-10V, edits the active sequence slot)");
		configInput(SEQ_CV_INPUT, "Sequence select CV (0.5 V per step: 0-10 V = slots 1-20; the SEQ knob follows)");

		configOutput(PITCH_OUTPUT, "Arpeggio pitch (1V/oct)");
		configOutput(GATE_OUTPUT, "Gate");
		configOutput(TRIG_OUTPUT, "Step trigger");
		configOutput(VEL_OUTPUT, "Velocity (0-10V)");
		configOutput(CHORD_OUTPUT, "Current chord (polyphonic)");
		configOutput(CHANGE_OUTPUT, "Chord change trigger");
		configOutput(LOOP_OUTPUT, "Progression loop trigger");

		initSlots();
	}

	// ------------------------------------------------------------------------
	// Slot helpers (all run under mtx)
	// ------------------------------------------------------------------------
	static void sanitizeSlot(SeqSlot &sl)
	{
		const auto &G = arplib::groups();
		sl.group = clamp(sl.group, 0, (int)G.size() - 1);
		sl.prog = clamp(sl.prog, 0, (int)G[sl.group].progs.size() - 1);
	}

	// Builds `staged` from the active slot (preset progression + the user's chord edits).
	// Caller must hold mtx.
	void rebuildStagedLocked()
	{
		const auto &G = arplib::groups();
		SeqSlot &sl = slots[activeSlot];
		sanitizeSlot(sl);
		groupIdx = sl.group;
		progIdx = sl.prog;

		Progression s;
		std::vector<std::string> toks = arplib::tokenize(G[sl.group].progs[sl.prog]);
		s.n = std::min((int)toks.size(), arplib::MAX_CHORDS);
		for (int i = 0; i < s.n; i++)
		{
			s.names[i] = sl.custom[i].empty() ? toks[i] : sl.custom[i];
			s.tones[i] = arplib::parseChord(s.names[i]);
		}
		staged = s;
		hasStaged = true;
		dispSeq = activeSlot + 1;
	}

	// Factory contents: slot k holds "Dark / Melancholic" progression #k.
	void initSlots()
	{
		std::lock_guard<std::mutex> lk(mtx);
		for (int i = 0; i < NUM_SEQ; i++)
		{
			slots[i].group = 0;
			slots[i].prog = i;
			for (auto &c : slots[i].custom)
				c.clear();
		}
		activeSlot = 0;
		rebuildStagedLocked();
	}

	// Choose the progression stored in the ACTIVE sequence slot.
	void setProgression(int g, int p)
	{
		std::lock_guard<std::mutex> lk(mtx);
		SeqSlot &sl = slots[activeSlot];
		sl.group = g;
		sl.prog = p;
		sanitizeSlot(sl);
		for (auto &c : sl.custom)
			c.clear();
		rebuildStagedLocked();
	}

	// Audio-thread slot switch; returns false (retry next sample) if the UI holds the lock.
	bool trySelectSlot(int idx)
	{
		std::unique_lock<std::mutex> lk(mtx, std::try_to_lock);
		if (!lk.owns_lock())
			return false;
		activeSlot = clamp(idx, 0, NUM_SEQ - 1);
		rebuildStagedLocked();
		return true;
	}

	// Menu text for a stored slot: left = number + chords (in the current key), right = style.
	void slotText(int i, int root, std::string &left, std::string &right)
	{
		const auto &G = arplib::groups();
		std::lock_guard<std::mutex> lk(mtx);
		const SeqSlot &sl = slots[clamp(i, 0, NUM_SEQ - 1)];
		std::vector<std::string> toks = arplib::tokenize(G[sl.group].progs[sl.prog]);
		left = std::to_string(i + 1) + ".  ";
		for (size_t t = 0; t < toks.size(); t++)
		{
			if (t)
				left += " - ";
			bool edited = t < (size_t)arplib::MAX_CHORDS && !sl.custom[t].empty();
			left += arplib::transposeName(edited ? sl.custom[t] : toks[t], root);
		}
		right = G[sl.group].name + " #" + std::to_string(sl.prog + 1);
	}

	// Current root in semitones (knob, or the CV-driven value when Root CV is patched).
	int rootNow()
	{
		int r = (int)params[ROOT_PARAM].getValue();
		if (inputs[ROOT_CV_INPUT].isConnected())
			r = dispTranspose;
		return arplib::mod12(r);
	}

	// Replace chord `slot` of the active sequence with a user-chosen chord (C-frame name).
	void setChord(int slot, const std::string &name)
	{
		std::lock_guard<std::mutex> lk(mtx);
		if (slot < 0 || slot >= staged.n)
			return;
		slots[activeSlot].custom[slot] = name;
		rebuildStagedLocked();
	}

	// Restore chord `slot` to the preset chord of the selected progression.
	void resetChord(int slot)
	{
		std::lock_guard<std::mutex> lk(mtx);
		if (slot < 0 || slot >= arplib::MAX_CHORDS)
			return;
		slots[activeSlot].custom[slot].clear();
		rebuildStagedLocked();
	}

	bool isCustom(int slot)
	{
		std::lock_guard<std::mutex> lk(mtx);
		return slot >= 0 && slot < arplib::MAX_CHORDS && !slots[activeSlot].custom[slot].empty();
	}

	std::string chordName(int slot)
	{
		std::lock_guard<std::mutex> lk(mtx);
		return (slot >= 0 && slot < staged.n) ? staged.names[slot] : std::string();
	}

	void onReset(const ResetEvent &e) override
	{
		Module::onReset(e);
		initSlots();
		doReset();
	}

	// Reset all sequencer counters so the next step starts a fresh chord.
	// `chordsSinceLoop` is also cleared so the RNDNEXT loop counter restarts
	// from zero and the first LOOP fires after `numChords` random advances.
	void doReset()
	{
		chordIdx = std::max(numChords - 1, 0);
		tickInChord = 1 << 30;
		patternStep = 0;
		walkIdx = 0;
		chordsSinceLoop = 0;
		tickCounter = 0;
		pending = false;
		dispTick = 0;
	}

	// ------------------------------------------------------------------------
	// Note pools
	// ------------------------------------------------------------------------
	// For every chord, stack all of its tones over `range` octaves, then
	// deduplicate and sort. `allPool` is the union across all chords, used by
	// the "random from whole progression" mode.
	void rebuildPools(int range)
	{
		allPool.n = 0;
		for (int c = 0; c < numChords; c++)
		{
			Pool &p = pools[c];
			p.n = 0;
			for (int o = 0; o < range; o++)
				for (int i = 0; i < chords[c].n; i++)
					if (p.n < POOL_MAX)
						p.note[p.n++] = chords[c].t[i] + 12 * o;
			std::sort(p.note, p.note + p.n);
			p.n = (int)(std::unique(p.note, p.note + p.n) - p.note);
			for (int i = 0; i < p.n && allPool.n < ALLPOOL_MAX; i++)
				allPool.note[allPool.n++] = p.note[i];
		}
		std::sort(allPool.note, allPool.note + allPool.n);
		allPool.n = (int)(std::unique(allPool.note, allPool.note + allPool.n) - allPool.note);
		poolRange = range;
		poolsDirty = false;
	}

	// Pattern -> pool index for the given step. `n` is the current pool size.
	int patternIndex(int mode, int step, int n)
	{
		if (n <= 1)
			return 0;
		switch (mode)
		{
		case PAT_UP:
			return step % n;
		case PAT_DOWN:
			return n - 1 - (step % n);
		case PAT_UPDOWN:
		{
			int per = 2 * n - 2;
			int i = step % per;
			return i < n ? i : per - i;
		}
		case PAT_DOWNUP:
		{
			int per = 2 * n - 2;
			int i = step % per;
			return n - 1 - (i < n ? i : per - i);
		}
		case PAT_CONVERGE:
		{
			int k = step % n;
			return (k % 2 == 0) ? k / 2 : n - 1 - k / 2;
		}
		case PAT_DIVERGE:
		{
			int k = n - 1 - (step % n);
			return (k % 2 == 0) ? k / 2 : n - 1 - k / 2;
		}
		case PAT_RANDOM:
			return (int)(random::u32() % (uint32_t)n);
		case PAT_WALK:
		{
			walkIdx += (random::u32() & 1) ? 1 : -1;
			if (walkIdx < 0)
				walkIdx = 1;
			if (walkIdx >= n)
				walkIdx = n - 2;
			return walkIdx;
		}
		case PAT_THUMB:
			return (step % 2 == 0) ? 0 : 1 + ((step / 2) % (n - 1));
		}
		return 0;
	}

	// ------------------------------------------------------------------------
	// Step firing
	// ------------------------------------------------------------------------
	// Called on each clock tick (or with `startDelay` when swing defers the
	// step). Handles chord advance (ordered or RNDNEXT-random), probability,
	// note choice, gate length, velocity and step-history publishing.
	void fireStep(float startDelay)
	{
		int stepsPerChord = stepOptions[clamp((int)params[STEPS_PARAM].getValue(), 0, NUM_STEP_OPTIONS - 1)];

		// ---- chord advance ----
		// In ordered mode the next chord is simply chordIdx + 1 mod n and LOOP
		// fires on every wrap. In RNDNEXT mode (unless `rndNextRepeat` allows
		// it) the next chord is picked uniformly from the other n-1 chords;
		// LOOP fires every n random advances so downstream loop counter
		// behaviour stays comparable to the ordered mode.
		if (tickInChord >= stepsPerChord)
		{
			const int nc = std::max(numChords, 1);
			const bool fresh = tickInChord >= (1 << 29); // first step after a reset: always chord 1
			const bool rndNext = params[RNDNEXT_PARAM].getValue() > 0.5f && !fresh && nc > 1;
			bool loop;
			if (rndNext)
			{
				if (rndNextRepeat)
					chordIdx = (int)(random::u32() % (uint32_t)nc);
				else
				{
					int nxt = (int)(random::u32() % (uint32_t)(nc - 1)); // any chord except the current one
					chordIdx = (nxt >= chordIdx) ? nxt + 1 : nxt;
				}
				loop = (++chordsSinceLoop >= nc); // "loop" = every nc random chord changes
				if (loop)
					chordsSinceLoop = 0;
			}
			else
			{
				chordIdx = (chordIdx + 1) % nc;
				loop = (chordIdx == 0);
				chordsSinceLoop = 0;
			}
			tickInChord = 0;
			patternStep = 0;
			walkIdx = 0;
			changePulse.trigger(1e-3f);
			if (loop)
				loopPulse.trigger(1e-3f);
			dispChord = chordIdx;
		}
		tickInChord++;
		dispTick = std::min(tickInChord, stepsPerChord);
		dispSteps = stepsPerChord;

		float rnd = clamp(params[RANDOM_PARAM].getValue() + inputs[RANDOM_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
		const Pool &pool = pools[chordIdx];
		if (pool.n <= 0)
			return;

		int step = patternStep++;

		// ---- note probability (rest) ----
		if (random::uniform() > params[PROB_PARAM].getValue())
		{
			gateRemain = 0.f;
			pushHist(currentNote, 0.f, true, false);
			return;
		}

		// ---- choose note ----
		int note;
		bool isRnd = false;
		if (rnd > 0.f && random::uniform() < rnd)
		{
			const Pool &rp = (randomFromChordOnly || allPool.n <= 0) ? pool : allPool;
			note = rp.note[random::u32() % (uint32_t)rp.n];
			isRnd = true;
		}
		else
		{
			note = pool.note[patternIndex((int)params[PATTERN_PARAM].getValue(), step, pool.n)];
		}
		currentNote = note;

		// ---- gate length: full tick, or /2 /4 /8 /16 depending on randomness ----
		int k = 0;
		if (rnd > 0.f && random::uniform() < rnd)
			k = (int)(random::u32() % 5);
		float len = period / (float)(1 << k) * params[GATE_PARAM].getValue();
		len = std::min(len, period - startDelay - 1e-3f); // always leave a re-trigger gap
		len = std::max(len, 5e-4f);
		gateRemain = len;
		trigPulse.trigger(1e-3f);
		pushHist(note, clamp(len / period, 0.f, 1.f), false, isRnd);

		// ---- velocity ----
		float human = params[HUMAN_PARAM].getValue();
		float v = (step == 0) ? 10.f : 8.f; // accent the first note of each chord
		v -= human * random::uniform() * 6.f;
		velocity = clamp(v, 0.f, 10.f);
	}

	// ------------------------------------------------------------------------
	// Per-sample processing
	// ------------------------------------------------------------------------
	void process(const ProcessArgs &args) override
	{
		float dt = args.sampleTime;

		// ---- pick up a newly selected progression ----
		if (hasStaged)
		{
			std::unique_lock<std::mutex> lk(mtx, std::try_to_lock);
			if (lk.owns_lock())
			{
				numChords = std::max(staged.n, 1);
				for (int i = 0; i < staged.n; i++)
					chords[i] = staged.tones[i];
				hasStaged = false;
				poolsDirty = true;
				if (chordIdx >= numChords)
					chordIdx = 0;
			}
		}
		int range = clamp((int)params[RANGE_PARAM].getValue(), 1, 4);
		if (poolsDirty || range != poolRange)
			rebuildPools(range);

		// ---- sequence slot: SEQ knob / SEQ CV (0.5 V per step, 0-10 V = slots 1-20, wraps) ----
		// With CV patched the CV selects the slot and the SEQ knob follows it; turning the knob by
		// hand still works and wins until the CV moves to another slot.
		{
			const bool cvOn = inputs[SEQ_CV_INPUT].isConnected();
			const int knob = clamp((int)std::round(params[SEQ_PARAM].getValue()) - 1, 0, NUM_SEQ - 1);
			int cvSlot = -1;
			int target = knob;
			if (cvOn)
			{
				cvSlot = (int)std::floor(inputs[SEQ_CV_INPUT].getVoltage() * 2.f + 1e-3f);
				cvSlot = ((cvSlot % NUM_SEQ) + NUM_SEQ) % NUM_SEQ;
				if (cvSlot != lastCvSlot)
					target = cvSlot; // CV moved: follow it
				else if (knob != lastKnobSlot)
					target = knob; // knob turned by hand
				else
					target = activeSlot; // nothing changed
			}
			bool ok = true;
			if (target != activeSlot)
				ok = trySelectSlot(target);
			if (ok)
			{
				if (cvOn)
				{
					lastCvSlot = cvSlot;
					if (knob != activeSlot)
						params[SEQ_PARAM].setValue((float)(activeSlot + 1)); // knob follows
				}
				else
					lastCvSlot = -1;
				lastKnobSlot = activeSlot;
			}
		}

		// ---- progression CV (edits the active slot) ----
		if (inputs[PROG_CV_INPUT].isConnected())
		{
			int idx = clamp((int)std::floor(inputs[PROG_CV_INPUT].getVoltage() / 10.f * 50.f), 0, 49);
			if (idx != lastProgCv)
			{
				lastProgCv = idx;
				if (idx != progIdx)
					setProgression(groupIdx, idx);
			}
		}
		else
			lastProgCv = -1;

		// ---- reset ----
		bool reset = resetTrig.process(inputs[RESET_INPUT].getVoltage(), 0.1f, 2.f);
		reset |= resetBtn.process(params[RESET_PARAM].getValue() > 0.5f);
		if (reset)
		{
			doReset();
			lights[RESET_LIGHT].setBrightness(1.f);
		}
		lights[RESET_LIGHT].setBrightnessSmooth(0.f, dt);

		// ---- clock ----
		// Measure the period between rising edges. Odd ticks are deferred by
		// `swing * period` to produce a shuffle feel.
		timeSinceClock += dt;
		if (clockTrig.process(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f))
		{
			if (clockSeen)
				period = clamp(timeSinceClock, 1e-3f, 10.f);
			clockSeen = true;
			timeSinceClock = 0.f;
			lights[CLOCK_LIGHT].setBrightness(1.f);

			uint64_t t = tickCounter++;
			float swing = params[SWING_PARAM].getValue();
			if ((t & 1) && swing > 0.f)
			{
				pending = true;
				pendingStartDelay = swing * period;
				pendingDelay = pendingStartDelay;
			}
			else
			{
				pending = false;
				fireStep(0.f);
			}
		}
		lights[CLOCK_LIGHT].setBrightnessSmooth(0.f, dt, 20.f);

		if (pending)
		{
			pendingDelay -= dt;
			if (pendingDelay <= 0.f)
			{
				pending = false;
				fireStep(pendingStartDelay);
			}
		}

		// ---- transposition ----
		int rootSemis = (int)params[ROOT_PARAM].getValue();
		rootSemis += (int)std::round(inputs[ROOT_CV_INPUT].getVoltage() * 12.f);
		int octSemis = 12 * (int)params[OCTAVE_PARAM].getValue();
		dispTranspose = ((rootSemis % 12) + 12) % 12;
		dispRandom = clamp(params[RANDOM_PARAM].getValue() + inputs[RANDOM_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);

		// ---- gate ----
		bool gate = gateRemain > 0.f;
		if (gate)
			gateRemain -= dt;

		// ---- outputs ----
		outputs[PITCH_OUTPUT].setVoltage((currentNote + rootSemis + octSemis) / 12.f);
		outputs[GATE_OUTPUT].setVoltage(gate ? 10.f : 0.f);
		outputs[TRIG_OUTPUT].setVoltage(trigPulse.process(dt) ? 10.f : 0.f);
		outputs[VEL_OUTPUT].setVoltage(velocity);
		outputs[CHANGE_OUTPUT].setVoltage(changePulse.process(dt) ? 10.f : 0.f);
		outputs[LOOP_OUTPUT].setVoltage(loopPulse.process(dt) ? 10.f : 0.f);

		const arplib::ChordTones &ct = chords[chordIdx];
		outputs[CHORD_OUTPUT].setChannels(ct.n);
		for (int i = 0; i < ct.n; i++)
			outputs[CHORD_OUTPUT].setVoltage((ct.t[i] + rootSemis + octSemis) / 12.f, i);

		lights[GATE_LIGHT].setBrightnessSmooth(gate ? 1.f : 0.f, dt, 40.f);
		lights[RNDNEXT_LIGHT].setBrightnessSmooth(params[RNDNEXT_PARAM].getValue() > 0.5f ? 1.f : 0.f, dt, 40.f);
	}

	// ------------------------------------------------------------------------
	// Persistence
	// ------------------------------------------------------------------------
	json_t *dataToJson() override
	{
		json_t *root = json_object();
		json_object_set_new(root, "randomFromChordOnly", json_boolean(randomFromChordOnly));
		json_object_set_new(root, "rndNextRepeat", json_boolean(rndNextRepeat));
		json_t *arr = json_array();
		{
			std::lock_guard<std::mutex> lk(mtx);
			for (int i = 0; i < NUM_SEQ; i++)
			{
				json_t *o = json_object();
				json_object_set_new(o, "group", json_integer(slots[i].group));
				json_object_set_new(o, "prog", json_integer(slots[i].prog));
				json_t *c = json_array();
				for (int k = 0; k < arplib::MAX_CHORDS; k++)
					json_array_append_new(c, json_string(slots[i].custom[k].c_str()));
				json_object_set_new(o, "custom", c);
				json_array_append_new(arr, o);
			}
		}
		json_object_set_new(root, "slots", arr);
		return root;
	}

	static void readCustom(json_t *c, SeqSlot &sl)
	{
		for (int k = 0; k < arplib::MAX_CHORDS; k++)
		{
			json_t *v = (c && json_is_array(c)) ? json_array_get(c, k) : nullptr;
			sl.custom[k] = (v && json_is_string(v)) ? json_string_value(v) : "";
		}
	}

	void dataFromJson(json_t *root) override
	{
		if (json_t *j = json_object_get(root, "randomFromChordOnly"))
			randomFromChordOnly = json_boolean_value(j);
		if (json_t *j = json_object_get(root, "rndNextRepeat"))
			rndNextRepeat = json_boolean_value(j);

		std::lock_guard<std::mutex> lk(mtx);
		json_t *arr = json_object_get(root, "slots");
		if (arr && json_is_array(arr))
		{
			for (int i = 0; i < NUM_SEQ; i++)
			{
				json_t *o = json_array_get(arr, i);
				if (!o || !json_is_object(o))
					continue;
				if (json_t *j = json_object_get(o, "group"))
					slots[i].group = (int)json_integer_value(j);
				if (json_t *j = json_object_get(o, "prog"))
					slots[i].prog = (int)json_integer_value(j);
				readCustom(json_object_get(o, "custom"), slots[i]);
				sanitizeSlot(slots[i]);
			}
		}
		else
		{
			// older patches stored one progression: it becomes sequence 1
			if (json_t *j = json_object_get(root, "group"))
				slots[0].group = (int)json_integer_value(j);
			if (json_t *j = json_object_get(root, "prog"))
				slots[0].prog = (int)json_integer_value(j);
			readCustom(json_object_get(root, "custom"), slots[0]);
			sanitizeSlot(slots[0]);
		}
		activeSlot = clamp((int)std::round(params[SEQ_PARAM].getValue()) - 1, 0, NUM_SEQ - 1);
		rebuildStagedLocked();
	}
};


// ============================================================================
//  PANEL LAYOUT  (px, 22 HP x 380)  -  all positions live here
// ============================================================================
namespace lay
{
	static const float W = 330.f;
	static const float H = 380.f;
	static const float M = 12.f; // side margin

	static const float DISP_Y = 26.f, DISP_H = 80.f; // live display (screen area)

	static const float Y_NOTES_HDR = 122.f, Y_NOTES = 146.f, NOTES_LABEL_DY = 21.f;
	static const float Y_FEEL_HDR = 176.f, Y_FEEL = 208.f;
	static const float FEEL_SMALL_LABEL_Y = 229.f, FEEL_LARGE_LABEL_Y = 239.f;

	static const float Y_IN_HDR = 250.f, IN_TOP = 258.f;
	static const float Y_OUT_HDR = 308.f, OUT_TOP = 316.f;
	static const float CELL_H = 40.f, JACK_DY = 27.f;

	// Column centres for a row of n controls spanning the panel width.
	inline float colN(int n, int i) { return M + (W - 2.f * M) / n * (i + 0.5f); }
	inline float col7(int i) { return colN(7, i); }
	inline float cellW(int n) { return (W - 2.f * M) / n - 4.f; }
	// FEEL row: one large knob (1.5 units) + four small knobs + the RNDNEXT switch cell (1 unit each)
	inline float feelUnit() { return (W - 2.f * M) / 6.5f; }
	inline float randomX() { return M + 0.75f * feelUnit(); }
	inline float feelX(int i) { return M + (2.f + (float)i) * feelUnit(); }
} // namespace lay

static const char *const PATTERN_NAMES[ArpLib::PAT_LEN] = {
	"UP", "DOWN", "UP-DOWN", "DOWN-UP", "CONVERGE", "DIVERGE", "RANDOM", "WALK", "THUMB"};

// Snapped round knob: value clicks to integer steps.
struct SmallSnapKnob : RoundSmallBlackKnob
{
	SmallSnapKnob() { snap = true; }
};


// ============================================================================
//  STATIC BACKDROP  (flat AGMM panel: gradient, bezel, headers, labels, jack cells)
// ============================================================================
// Only draws things that don't change at audio rate; the live display and
// knob arcs are separate widgets stacked on top of this.
struct ArpBackdrop : TransparentWidget
{
	void draw(const DrawArgs &args) override
	{
		NVGcontext *vg = args.vg;
		const float W = box.size.x;
		using namespace agmm;

		panelBase(vg, W, box.size.y, "ARPLIB");
		bezel(vg, lay::M, lay::DISP_Y, W - 2.f * lay::M, lay::DISP_H);

		// ---- NOTES ----
		sectionHeader(vg, W, lay::Y_NOTES_HDR, "NOTES");
		const char *nl[7] = {"ROOT", "OCT", "RANGE", "PATTERN", "STEPS", "SEQ", "RESET"};
		for (int i = 0; i < 7; i++)
			text(vg, lay::col7(i), lay::Y_NOTES + lay::NOTES_LABEL_DY, nl[i], 8.f, i == 5 ? bright() : dim(), ALIGN_C, 0.5f);

		// ---- FEEL (RANDOM large, four small knobs, then the RNDNEXT switch cell) ----
		sectionHeader(vg, W, lay::Y_FEEL_HDR, "FEEL");
		text(vg, lay::randomX(), lay::FEEL_LARGE_LABEL_Y, "RANDOM", 8.f, violet(), ALIGN_C, 0.5f);
		const char *fl[5] = {"PROB", "GATE", "SWING", "HUMAN", "RNDNEXT"};
		for (int i = 0; i < 5; i++)
			text(vg, lay::feelX(i), lay::FEEL_SMALL_LABEL_Y, fl[i], 8.f, dim(), ALIGN_C, 0.5f);

		// ---- INPUTS ----
		sectionHeader(vg, W, lay::Y_IN_HDR, "INPUTS");
		const char *il[6] = {"CLOCK", "RESET", "ROOT CV", "RND CV", "PROG CV", "SEQ CV"};
		for (int i = 0; i < 6; i++)
			cell(vg, lay::colN(6, i), lay::IN_TOP, lay::cellW(6), lay::CELL_H, il[i], false, 8.f);

		// ---- OUTPUTS (tinted) ----
		sectionHeader(vg, W, lay::Y_OUT_HDR, "OUTPUTS");
		const char *ol[7] = {"PITCH", "GATE", "TRIG", "VEL", "CHORD", "CHG", "LOOP"};
		for (int i = 0; i < 7; i++)
			cell(vg, lay::colN(7, i), lay::OUT_TOP, lay::cellW(7), lay::CELL_H, ol[i], true, 8.f);

		Widget::draw(args);
	}
};


// ============================================================================
//  LIVE DISPLAY: chord strip with progress + 16-step piano roll
// ============================================================================
// The chord strip shows the current progression (transposed to the current
// root). Clicking a chord opens a circle-of-fifths drop-down to override it.
// The piano roll at the bottom shows the last 16 steps: horizontal position
// is time, vertical position is pitch, bar length is gate length. Random
// notes are tinted violet, rests appear as a thin dash at the bottom.
struct ProgDisplay : Widget
{
	ArpLib *module = nullptr;
	int hoverSlot = -1;

	// chord strip geometry (shared by drawing and hit-testing)
	static constexpr float STRIP_X = 6.f, STRIP_Y = 17.f, STRIP_H = 24.f, STRIP_GAP = 3.f;
	static float slotWidth(int n, float w)
	{
		return (w - 2.f * STRIP_X - (n - 1) * STRIP_GAP) / (float)(n < 1 ? 1 : n);
	}

	// Returns the chord-strip slot under `pos`, or -1 if outside.
	int hitSlot(Vec pos)
	{
		if (!module)
			return -1;
		int n;
		{
			std::lock_guard<std::mutex> lk(module->mtx);
			n = module->staged.n;
		}
		float cw = slotWidth(n, box.size.x);
		for (int i = 0; i < n; i++)
		{
			float x = STRIP_X + i * (cw + STRIP_GAP);
			if (pos.x >= x && pos.x <= x + cw && pos.y >= STRIP_Y && pos.y <= STRIP_Y + STRIP_H)
				return i;
		}
		return -1;
	}

	void onHover(const HoverEvent &e) override
	{
		hoverSlot = hitSlot(e.pos);
		e.consume(this);
	}
	void onLeave(const LeaveEvent &e) override
	{
		hoverSlot = -1;
	}

	void onButton(const ButtonEvent &e) override
	{
		if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT)
		{
			int slot = hitSlot(e.pos);
			if (slot >= 0)
			{
				openChordMenu(slot);
				e.consume(this);
				return;
			}
		}
		Widget::onButton(e); // right-click etc. fall through to the module menu
	}

	// Drop-down: all 12 circle-of-fifths roots, each with its chord qualities.
	// Chords are shown in the current key; they are stored relative to C.
	void openChordMenu(int slot)
	{
		ArpLib *mod = module;
		if (!mod)
			return;
		ui::Menu *menu = createMenu();
		menu->addChild(createMenuLabel("Chord " + std::to_string(slot + 1) + " - circle of fifths"));
		if (mod->isCustom(slot))
			menu->addChild(createMenuItem("Reset to preset chord", "", [=]()
										  { mod->resetChord(slot); }));
		menu->addChild(new ui::MenuSeparator);

		const int root = mod->rootNow();
		const std::string curStored = mod->chordName(slot);
		int curRootPC = -1, curBass;
		std::string curSuffix;
		arplib::splitChord(curStored, curRootPC, curSuffix, curBass);

		for (int k = 0; k < 12; k++)
		{
			const int pc = arplib::CIRCLE_OF_FIFTHS[k];
			const int rel = arplib::mod12(pc - root);
			const bool here = (rel == curRootPC);
			menu->addChild(createSubmenuItem(arplib::pcLabel(pc), here ? "\xe2\x80\xa2" : "", [=](ui::Menu *m)
											 {
				for (int q = 0; q < arplib::NUM_QUALITIES; q++) {
					const std::string suffix = arplib::QUALITIES[q].suffix;
					const std::string stored = std::string(arplib::FLAT_NAMES[rel]) + suffix;
					const std::string shown = std::string(arplib::FLAT_NAMES[pc]) + suffix;
					m->addChild(createCheckMenuItem(shown, arplib::QUALITIES[q].desc,
						[=]() { return arplib::sameChord(mod->chordName(slot), stored); },
						[=]() { mod->setChord(slot, stored); }));
				} }));
		}
	}

	// Background (screen bezel) is drawn on layer 0 so the content can be
	// drawn on layer 1 and stay bright when the room is dimmed.
	void draw(const DrawArgs &args) override
	{
		agmm::screen(args.vg, 0.f, 0.f, box.size.x, box.size.y);
		Widget::draw(args);
	}

	void drawLayer(const DrawArgs &args, int layer) override
	{
		if (layer == 1)
			drawContent(args.vg); // self-lit, stays bright in a dimmed room
		Widget::drawLayer(args, layer);
	}

	void drawContent(NVGcontext *vg)
	{
		using namespace agmm;
		const float w = box.size.x, h = box.size.y;

		// ---- gather state (live module, or a demo for the module browser) ----
		std::string title = "Dark / Melancholic  #1";
		std::vector<std::string> names = {"Cm", "Ab", "Eb", "Bb"};
		bool cust[arplib::MAX_CHORDS] = {};
		int cur = 0, tick = 3, steps = 8, pat = 2;
		float rnd = 0.25f;
		int seq = 1;
		bool clk = false, gate = true;
		ArpLib::StepRec hist[ArpLib::HIST_LEN];
		int head = 0;

		if (module)
		{
			{
				std::lock_guard<std::mutex> lk(module->mtx);
				const auto &G = arplib::groups();
				title = G[module->groupIdx].name + "  #" + std::to_string(module->progIdx + 1);
				int tr = module->dispTranspose;
				names.clear();
				for (int i = 0; i < module->staged.n; i++)
				{
					names.push_back(arplib::transposeName(module->staged.names[i], tr));
					cust[i] = !module->slots[module->activeSlot].custom[i].empty();
				}
			}
			cur = module->dispChord;
			tick = module->dispTick;
			steps = std::max((int)module->dispSteps, 1);
			pat = clamp((int)module->params[ArpLib::PATTERN_PARAM].getValue(), 0, ArpLib::PAT_LEN - 1);
			rnd = module->dispRandom;
			seq = module->dispSeq;
			clk = module->lights[ArpLib::CLOCK_LIGHT].getBrightness() > 0.3f;
			gate = module->lights[ArpLib::GATE_LIGHT].getBrightness() > 0.3f;
			head = module->histHead;
			for (int i = 0; i < ArpLib::HIST_LEN; i++)
				hist[i] = module->hist[i];
		}
		else
		{
			// demo data for the module browser: no module attached
			static const int demo[8] = {0, 4, 7, 12, 7, 4, 0, -5};
			for (int i = 0; i < ArpLib::HIST_LEN; i++)
			{
				hist[i].valid = true;
				hist[i].note = demo[i % 8];
				hist[i].len = (i % 5 == 3) ? 0.5f : 0.95f;
				hist[i].rnd = (i % 7 == 4);
				hist[i].rest = (i == 9);
			}
		}

		// ---- header: group / index on the left, status chips on the right ----
		text(vg, 6.f, 8.f, title.c_str(), 8.f, dim(), ALIGN_L, 0.5f);

		char seqBuf[16];
		snprintf(seqBuf, sizeof(seqBuf), "SEQ %02d", seq);
		struct Chip
		{
			const char *label;
			float w;
			bool on;
			NVGcolor col;
		};
		Chip chips[4] = {
			// drawn right-to-left
			{"CLK", 26.f, clk, accent()},
			{"GATE", 32.f, gate, warm()},
			{"RND", 26.f, rnd > 0.001f, violet()},
			{seqBuf, 46.f, true, bright()}, // left of RND
		};
		float cx = w - 6.f;
		for (const Chip &c : chips)
		{
			cx -= c.w;
			chip(vg, cx, 3.f, c.w, 11.f, c.label, c.on, c.col);
			cx -= 3.f;
		}

		// ---- chord strip ----
		const float sy = STRIP_Y, sh = STRIP_H, sx = STRIP_X;
		int n = std::max((int)names.size(), 1);
		float cw = slotWidth(n, w);
		for (int i = 0; i < (int)names.size(); i++)
		{
			float x = sx + i * (cw + STRIP_GAP);
			bool active = (i == cur);
			nvgBeginPath(vg);
			nvgRoundedRect(vg, x, sy, cw, sh, 3.f);
			if (active)
			{
				nvgFillColor(vg, withA(accent(), 0.18f));
				nvgFill(vg);
				nvgStrokeColor(vg, withA(accent(), 0.9f));
			}
			else
				nvgStrokeColor(vg, nvgRGBA(255, 255, 255, hoverSlot == i ? 120 : 30));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			// drop-down caret (top-right) and "edited" dot (top-left)
			nvgBeginPath(vg);
			nvgMoveTo(vg, x + cw - 9.f, sy + 5.f);
			nvgLineTo(vg, x + cw - 4.f, sy + 5.f);
			nvgLineTo(vg, x + cw - 6.5f, sy + 8.f);
			nvgClosePath(vg);
			nvgFillColor(vg, withA(dim(), hoverSlot == i ? 1.f : 0.55f));
			nvgFill(vg);
			if (cust[i])
			{
				nvgBeginPath(vg);
				nvgCircle(vg, x + 5.f, sy + 6.f, 1.8f);
				nvgFillColor(vg, violet());
				nvgFill(vg);
			}

			float fs = std::min(11.f, (cw - 6.f) / (std::max((int)names[i].size(), 1) * 0.62f));
			text(vg, x + cw / 2.f, sy + sh / 2.f - 2.f, names[i].c_str(), fs, active ? bright() : dim(), ALIGN_C, 0.f);

			if (active)
			{
				float prog = clamp((float)tick / (float)steps, 0.f, 1.f);
				nvgBeginPath(vg);
				nvgRect(vg, x + 3.f, sy + sh - 5.f, (cw - 6.f) * prog, 2.f);
				nvgFillColor(vg, accent());
				nvgFill(vg);
			}
		}

		// ---- piano roll: last 16 steps, oldest on the left ----
		const float px = 6.f, py = 46.f, pw = w - 12.f, ph = h - py - 5.f;
		plotFrame(vg, px, py, pw, ph);
		text(vg, px + pw - 4.f, py + 8.f, PATTERN_NAMES[pat], 7.f, withA(dim(), 0.7f), ALIGN_R, 1.f);

		const int N = ArpLib::HIST_LEN;
		const float colW = pw / N;
		// faint gridlines every 4 steps for rhythmic orientation
		for (int i = 4; i < N; i += 4)
		{
			nvgBeginPath(vg);
			nvgMoveTo(vg, px + i * colW, py + 1.f);
			nvgLineTo(vg, px + i * colW, py + ph - 1.f);
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 14));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);
		}

		// work out the vertical range from the steps actually shown
		int lo = 1000, hi = -1000;
		for (int i = 0; i < N; i++)
		{
			const ArpLib::StepRec &r = hist[(head + i) % N];
			if (r.valid && !r.rest)
			{
				lo = std::min(lo, r.note);
				hi = std::max(hi, r.note);
			}
		}
		if (lo > hi)
		{
			lo = 0;
			hi = 12;
		}
		if (hi - lo < 12)
		{
			int mid = (hi + lo) / 2;
			lo = mid - 6;
			hi = mid + 6;
		}

		// newest-step column highlight
		nvgBeginPath(vg);
		nvgRect(vg, px + (N - 1) * colW, py + 1.f, colW, ph - 2.f);
		nvgFillColor(vg, withA(accent(), 0.10f));
		nvgFill(vg);

		for (int i = 0; i < N; i++)
		{
			const ArpLib::StepRec &r = hist[(head + i) % N];
			if (!r.valid)
				continue;
			float x = px + i * colW + 1.f;
			if (r.rest)
			{
				nvgBeginPath(vg);
				nvgRect(vg, x, py + ph - 3.5f, colW - 2.f, 1.5f);
				nvgFillColor(vg, withA(dim(), 0.5f));
				nvgFill(vg);
				continue;
			}
			float ny = py + ph - 5.f - (float)(r.note - lo) / (float)(hi - lo) * (ph - 10.f);
			float bw = std::max(colW * r.len - 1.5f, 2.f);
			float fade = 0.35f + 0.65f * (float)i / (float)(N - 1);
			NVGcolor col = r.rnd ? violet() : (i == N - 1 ? bright() : accent());
			nvgBeginPath(vg);
			nvgRect(vg, x, ny - 1.5f, bw, 3.f);
			nvgFillColor(vg, withA(col, fade));
			nvgFill(vg);
		}
	}
};


// ============================================================================
//  KNOB GAUGES: circular value arcs around every knob (same look as XBodyProblem)
// ============================================================================
// Drawn on top of the backdrop but below the knobs. Each gauge reads its
// parameter's scaled value and paints an arc from the arc start (or centre
// for bipolar controls) to the current value.
struct ArpKnobArcs : TransparentWidget
{
	ArpLib *module = nullptr;

	struct Gauge
	{
		int param;
		float x, y, r;
		bool bipolar;
		bool violetArc;
	};
	std::vector<Gauge> gauges;

	ArpKnobArcs(ArpLib *m) : module(m)
	{
		box.size = Vec(lay::W, lay::H);
		const float rs = 16.f, rl = 26.f;
		gauges = {
			{ArpLib::ROOT_PARAM, lay::col7(0), lay::Y_NOTES, rs, false, false},
			{ArpLib::OCTAVE_PARAM, lay::col7(1), lay::Y_NOTES, rs, true, false},
			{ArpLib::RANGE_PARAM, lay::col7(2), lay::Y_NOTES, rs, false, false},
			{ArpLib::PATTERN_PARAM, lay::col7(3), lay::Y_NOTES, rs, false, false},
			{ArpLib::STEPS_PARAM, lay::col7(4), lay::Y_NOTES, rs, false, false},
			{ArpLib::SEQ_PARAM, lay::col7(5), lay::Y_NOTES, rs, false, false},
			{ArpLib::RANDOM_PARAM, lay::randomX(), lay::Y_FEEL, rl, false, true},
			{ArpLib::PROB_PARAM, lay::feelX(0), lay::Y_FEEL, rs, false, false},
			{ArpLib::GATE_PARAM, lay::feelX(1), lay::Y_FEEL, rs, false, false},
			{ArpLib::SWING_PARAM, lay::feelX(2), lay::Y_FEEL, rs, false, false},
			{ArpLib::HUMAN_PARAM, lay::feelX(3), lay::Y_FEEL, rs, false, false},
		};
	}

	void draw(const DrawArgs &args) override
	{
		NVGcontext *vg = args.vg;
		// Arc start is 45 degrees past straight-up on the left, end 45 past on the right.
		const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
		const float a1 = 0.75f * (float)M_PI - (float)M_PI / 2.f;

		for (const Gauge &g : gauges)
		{
			// track
			nvgBeginPath(vg);
			nvgArc(vg, g.x, g.y, g.r, a0, a1, NVG_CW);
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
			nvgStrokeWidth(vg, 2.f);
			nvgLineCap(vg, NVG_ROUND);
			nvgStroke(vg);

			if (!module)
				continue;
			ParamQuantity *pq = module->paramQuantities[g.param];
			if (!pq)
				continue;
			float v = pq->getScaledValue();
			float from = g.bipolar ? 0.5f : 0.f;
			float lo = std::min(from, v), hi = std::max(from, v);
			if (hi - lo < 0.004f)
				continue;

			NVGcolor col = g.violetArc				 ? agmm::violet()
						   : (g.bipolar && v < 0.5f) ? agmm::warm()
													 : agmm::accent();
			nvgBeginPath(vg);
			nvgArc(vg, g.x, g.y, g.r, a0 + (a1 - a0) * lo, a0 + (a1 - a0) * hi, NVG_CW);
			nvgStrokeColor(vg, col);
			nvgStrokeWidth(vg, 2.f);
			nvgStroke(vg);
		}
	}
};


// ============================================================================
//  WIDGET
// ============================================================================
struct ArpLibWidget : ModuleWidget
{
	ArpLibWidget(ArpLib *module)
	{
		setModule(module);
		box.size = Vec(lay::W, lay::H); // 22 HP; the whole panel is drawn in code (no SVG)

		ArpBackdrop *bg = new ArpBackdrop;
		bg->box.size = box.size;
		addChild(bg);
		addChild(new ArpKnobArcs(module)); // gauges sit under the knobs

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		ProgDisplay *disp = new ProgDisplay;
		disp->module = module;
		disp->box.pos = Vec(lay::M, lay::DISP_Y);
		disp->box.size = Vec(lay::W - 2.f * lay::M, lay::DISP_H);
		addChild(disp);

		// ---- NOTES row ----
		addParam(createParamCentered<SmallSnapKnob>(Vec(lay::col7(0), lay::Y_NOTES), module, ArpLib::ROOT_PARAM));
		addParam(createParamCentered<SmallSnapKnob>(Vec(lay::col7(1), lay::Y_NOTES), module, ArpLib::OCTAVE_PARAM));
		addParam(createParamCentered<SmallSnapKnob>(Vec(lay::col7(2), lay::Y_NOTES), module, ArpLib::RANGE_PARAM));
		addParam(createParamCentered<SmallSnapKnob>(Vec(lay::col7(3), lay::Y_NOTES), module, ArpLib::PATTERN_PARAM));
		addParam(createParamCentered<SmallSnapKnob>(Vec(lay::col7(4), lay::Y_NOTES), module, ArpLib::STEPS_PARAM));
		addParam(createParamCentered<SmallSnapKnob>(Vec(lay::col7(5), lay::Y_NOTES), module, ArpLib::SEQ_PARAM));
		addParam(createLightParamCentered<VCVLightButton<MediumSimpleLight<agmm::CyanLight>>>(
			Vec(lay::col7(6), lay::Y_NOTES), module, ArpLib::RESET_PARAM, ArpLib::RESET_LIGHT));

		// ---- FEEL row ----
		addParam(createParamCentered<RoundLargeBlackKnob>(Vec(lay::randomX(), lay::Y_FEEL), module, ArpLib::RANDOM_PARAM));
		addParam(createParamCentered<RoundSmallBlackKnob>(Vec(lay::feelX(0), lay::Y_FEEL), module, ArpLib::PROB_PARAM));
		addParam(createParamCentered<RoundSmallBlackKnob>(Vec(lay::feelX(1), lay::Y_FEEL), module, ArpLib::GATE_PARAM));
		addParam(createParamCentered<RoundSmallBlackKnob>(Vec(lay::feelX(2), lay::Y_FEEL), module, ArpLib::SWING_PARAM));
		addParam(createParamCentered<RoundSmallBlackKnob>(Vec(lay::feelX(3), lay::Y_FEEL), module, ArpLib::HUMAN_PARAM));

		// RNDNEXT switch + indicator light (rightmost FEEL cell)
		addParam(createParamCentered<CKSS>(Vec(lay::feelX(4) - 7.f, lay::Y_FEEL), module, ArpLib::RNDNEXT_PARAM));
		addChild(createLightCentered<SmallLight<agmm::VioletLight>>(Vec(lay::feelX(4) + 11.f, lay::Y_FEEL), module, ArpLib::RNDNEXT_LIGHT));

		// ---- inputs ----
		const float inY = lay::IN_TOP + lay::JACK_DY;
		addInput(createInputCentered<PJ301MPort>(Vec(lay::colN(6, 0), inY), module, ArpLib::CLOCK_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(lay::colN(6, 1), inY), module, ArpLib::RESET_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(lay::colN(6, 2), inY), module, ArpLib::ROOT_CV_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(lay::colN(6, 3), inY), module, ArpLib::RANDOM_CV_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(lay::colN(6, 4), inY), module, ArpLib::PROG_CV_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(lay::colN(6, 5), inY), module, ArpLib::SEQ_CV_INPUT));

		// ---- outputs ----
		const float outY = lay::OUT_TOP + lay::JACK_DY;
		addOutput(createOutputCentered<PJ301MPort>(Vec(lay::colN(7, 0), outY), module, ArpLib::PITCH_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(Vec(lay::colN(7, 1), outY), module, ArpLib::GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(Vec(lay::colN(7, 2), outY), module, ArpLib::TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(Vec(lay::colN(7, 3), outY), module, ArpLib::VEL_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(Vec(lay::colN(7, 4), outY), module, ArpLib::CHORD_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(Vec(lay::colN(7, 5), outY), module, ArpLib::CHANGE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(Vec(lay::colN(7, 6), outY), module, ArpLib::LOOP_OUTPUT));
		agmm::addKnobArcs(this, 6.f);
	}

	// Context menu: progression picker (minor / major categories), slot
	// selection, the RNDNEXT repeat behaviour, and the "random from current
	// chord only" toggle.
	void appendContextMenu(Menu *menu) override
	{
		ArpLib *module = getModule<ArpLib>();
		if (!module)
			return;

		menu->addChild(new MenuSeparator);
		menu->addChild(createMenuLabel("Chord progression  (SEQ " + std::to_string(module->activeSlot + 1) + ")"));

		auto addCategory = [=](Menu *parent, const std::string &label, int firstGroup, int count)
		{
			parent->addChild(createSubmenuItem(label, "", [=](Menu *catMenu)
											   {
				const auto& G = arplib::groups();
				for (int gi = firstGroup; gi < firstGroup + count; gi++) {
					std::string right = (module->groupIdx == gi) ? "#" + std::to_string(module->progIdx + 1) : "";
					catMenu->addChild(createSubmenuItem(G[gi].name, right, [=](Menu* gMenu) {
						const auto& GG = arplib::groups();
						const int root = module->rootNow();   // read when the menu opens
						for (int pi = 0; pi < (int) GG[gi].progs.size(); pi++) {
							std::string text = std::to_string(pi + 1) + ".  ";
							std::vector<std::string> toks = arplib::tokenize(GG[gi].progs[pi]);
							for (size_t t = 0; t < toks.size(); t++) {
								if (t) text += " - ";
								text += arplib::transposeName(toks[t], root);
							}
							gMenu->addChild(createCheckMenuItem(text, "",
								[=]() { return module->groupIdx == gi && module->progIdx == pi; },
								[=]() { module->setProgression(gi, pi); }));
						}
					}));
				} }));
		};
		const std::string rootName = arplib::FLAT_NAMES[module->rootNow()];
		addCategory(menu, "Minor (" + rootName + " minor)", 0, arplib::NUM_MINOR_GROUPS);
		addCategory(menu, "Major (" + rootName + " major)", arplib::NUM_MINOR_GROUPS, arplib::NUM_GROUPS - arplib::NUM_MINOR_GROUPS);
		menu->addChild(createSubmenuItem("Sequence slots (1-20)", "", [=](Menu *m)
										 {
			const int root = module->rootNow();
			for (int i = 0; i < ArpLib::NUM_SEQ; i++) {
				std::string left, right;
				module->slotText(i, root, left, right);
				m->addChild(createCheckMenuItem(left, right,
					[=]() { return module->activeSlot == i; },
					[=]() { module->params[ArpLib::SEQ_PARAM].setValue((float) (i + 1)); }));
			} }));
		menu->addChild(createMenuLabel("Tip: click a chord on the display to change it"));
		menu->addChild(createMenuItem("Reset edited chords to preset", "", [=]()
									  { module->setProgression(module->groupIdx, module->progIdx); }));

		menu->addChild(new MenuSeparator);
		menu->addChild(createBoolPtrMenuItem("Random notes from current chord only", "", &module->randomFromChordOnly));
		menu->addChild(createBoolPtrMenuItem("RNDNEXT may repeat the same chord", "", &module->rndNextRepeat));
	}
};

Model *modelArpLib = createModel<ArpLib, ArpLibWidget>("ArpLib");