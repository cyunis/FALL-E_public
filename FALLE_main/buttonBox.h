#ifndef BUTTON_H
#define BUTTON_H

namespace buttonBox {

  struct BUTTONINFO {
    bool syncBool; //black button
    bool startBool; //white button
  };

  extern BUTTONINFO buttonInfo;

  void readPin(unsigned long duration);

}  // namespace buttonBox

#endif
