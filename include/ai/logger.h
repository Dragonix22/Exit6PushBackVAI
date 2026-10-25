#pragma once
/*---------------------------------------------------------------------------*/
/*  SD-card logging.                                                         */
/*                                                                           */
/*  USB is taken by the Jetson, so printf debugging is not available (and    */
/*  printf output would be sent to the Jetson). Log to the Brain's SD card   */
/*  instead, then pull the CSV off the card after a match and replay it.     */
/*                                                                           */
/*  Each power-up writes a new file: ai_000.csv, ai_001.csv, ...             */
/*  Lines are "time_ms,TYPE,..." so they are easy to load in Python.         */
/*---------------------------------------------------------------------------*/

namespace ai {
namespace log {

void init();                                // call once in pre_auton
void event(const char *fmt, ...);           // free text line, type EV
void pose_row(float x, float y, float h, int gps_q, float gps_err, int task);
void flush();                               // runs automatically every 0.5 s
bool enabled();
const char *file_name();

}  // namespace log
}  // namespace ai
