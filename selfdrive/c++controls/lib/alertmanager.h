#ifndef ALERTMANAGER_H_
#define ALERTMANAGER_H_
#include <string>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"

enum Priority
{
    LOWEST, 
    LOW_LOWEST, 
    LOW, 
    MID, 
    HIGH, 
    HIGHEST
}; 

typedef struct
{
    std::string alert_type;
    std::string alert_text_1;
    std::string alert_text_2;
    cereal::ControlsState::AlertStatus alert_status;
    cereal::ControlsState::AlertSize alert_size;
    int alert_priority;
    cereal::CarControl::HUDControl::VisualAlert visual_alert;
    cereal::CarControl::HUDControl::AudibleAlert audible_alert;
    duration_sound;
    duration_hud_alert;
    duration_text;
    start_time;
    alert_rate;
}Alert;

class AlertManager
{
  void __init__(self):
    self.activealerts = []
    self.alerts = {alert.alert_type: alert for alert in ALERTS}

  void alertPresent(self):
    return len(self.activealerts) > 0

  void add(self, frame, alert_type, enabled=True, extra_text_1='', extra_text_2=''):
    alert_type = str(alert_type)
    added_alert = copy.copy(self.alerts[alert_type])
    added_alert.alert_text_1 += extra_text_1
    added_alert.alert_text_2 += extra_text_2
    added_alert.start_time = frame * DT_CTRL

    // if new alert is higher priority, log it
    if not self.alertPresent() or added_alert.alert_priority > self.activealerts[0].alert_priority:
          cloudlog.event('alert_add', alert_type=alert_type, enabled=enabled)

    self.activealerts.append(added_alert)

    // sort by priority first and then by start_time
    self.activealerts.sort(key=lambda k: (k.alert_priority, k.start_time), reverse=True)

  void process_alerts(self, frame):
    cur_time = frame * DT_CTRL

    // first get rid of all the expired alerts
    self.activealerts = [a for a in self.activealerts if a.start_time +
   max(a.duration_sound, a.duration_hud_alert, a.duration_text) > cur_time]

    current_alert = self.activealerts[0] if self.alertPresent() else None

    // start with assuming no alerts
    self.alert_type = ""
    self.alert_text_1 = ""
    self.alert_text_2 = ""
    self.alert_status = AlertStatus.normal
    self.alert_size = AlertSize.none
    self.visual_alert = VisualAlert.none
    self.audible_alert = AudibleAlert.none
    self.alert_rate = 0.

    if current_alert:
      self.alert_type = current_alert.alert_type

      if current_alert.start_time + current_alert.duration_sound > cur_time:
        self.audible_alert = current_alert.audible_alert

      if current_alert.start_time + current_alert.duration_hud_alert > cur_time:
        self.visual_alert = current_alert.visual_alert

      if current_alert.start_time + current_alert.duration_text > cur_time:
        self.alert_text_1 = current_alert.alert_text_1
        self.alert_text_2 = current_alert.alert_text_2
        self.alert_status = current_alert.alert_status
        self.alert_size = current_alert.alert_size
        self.alert_rate = current_alert.alert_rate
};

#endif //ALERTMANAGER_H_