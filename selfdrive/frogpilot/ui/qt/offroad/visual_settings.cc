#include "selfdrive/frogpilot/ui/qt/offroad/visual_settings.h"

FrogPilotVisualsPanel::FrogPilotVisualsPanel(FrogPilotSettingsWindow *parent) : FrogPilotListWidget(parent) {
  const std::vector<std::tuple<QString, QString, QString, QString>> visualToggles {
    {"BonusContent", tr("Bonus Content"), tr("Bonus FrogPilot features to make openpilot a bit more fun!"), "../frogpilot/assets/toggle_icons/frog.png"},
    {"GoatScream", tr("Goat Scream"), tr("Enable the famed 'Goat Scream' that has brought both joy and anger to FrogPilot users all around the world!"), ""},
    {"HolidayThemes", tr("Holiday Themes"), tr("The openpilot theme changes according to the current/upcoming holiday. Minor holidays last a day, while major holidays (Easter, Christmas, Halloween, etc.) last a week."), ""},
    {"PersonalizeOpenpilot", tr("Personalize openpilot"), tr("Customize openpilot to your personal tastes!"), ""},
    {"CustomColors", tr("Color Theme"), tr("Switch out the standard openpilot color scheme with themed colors.\n\nWant to submit your own color scheme? Post it in the 'feature-request' channel in the FrogPilot Discord!"), ""},
    {"CustomIcons", tr("Icon Pack"), tr("Switch out the standard openpilot icons with a set of themed icons.\n\nWant to submit your own icon pack? Post it in the 'feature-request' channel in the FrogPilot Discord!"), ""},
    {"CustomSounds", tr("Sound Pack"), tr("Switch out the standard openpilot sounds with a set of themed sounds.\n\nWant to submit your own sound pack? Post it in the 'feature-request' channel in the FrogPilot Discord!"), ""},
    {"CustomSignals", tr("Turn Signals"), tr("Add themed animation for your turn signals.\n\nWant to submit your own turn signal animation? Post it in the 'feature-request' channel in the FrogPilot Discord!"), ""},
    {"WheelIcon", tr("Steering Wheel"), tr("Replace the default steering wheel icon with a custom icon."), ""},
    {"DownloadStatusLabel", tr("Download Status"), "", ""},
    {"StartupAlert", tr("Startup Alert"), tr("Customize the 'Startup' alert that is shown when you go onroad."), ""},
    {"RandomEvents", tr("Random Events"), tr("Enjoy a bit of unpredictability with random events that can occur during certain driving conditions. This is purely cosmetic and has no impact on driving controls!"), ""},

    {"CustomUI", tr("Custom Onroad UI"), tr("Customize the Onroad UI."), "../assets/offroad/icon_road.png"},
    {"Compass", tr("Compass"), tr("Add a compass to the onroad UI."), ""},
    {"CustomPaths", tr("Paths"), tr("Show your projected acceleration on the driving path, detected adjacent lanes, or when a vehicle is detected in your blindspot."), ""},
    {"PedalsOnUI", tr("Pedals Being Pressed"), tr("Display the brake and gas pedals on the onroad UI below the steering wheel icon."), ""},
    {"RoadNameUI", tr("Road Name"), tr("Display the current road's name at the bottom of the screen. Sourced from OpenStreetMap."), ""},
    {"RotatingWheel", tr("Rotating Steering Wheel"), tr("Rotate the steering wheel in the onroad UI alongside your physical steering wheel."), ""},
    {"ShowStoppingPoint", tr("Stopping Points"), tr("Display the point where openpilot wants to stop for red lights/stop signs."), ""},

    {"QOLVisuals", tr("Quality of Life"), tr("Miscellaneous quality of life changes to improve your overall openpilot experience."), "../frogpilot/assets/toggle_icons/quality_of_life.png"},
    {"BigMap", tr("Big Map"), tr("Increase the size of the map in the onroad UI."), ""},
    {"CameraView", tr("Camera View"), tr("Choose your preferred camera view for the onroad UI. This is purely a visual change and doesn't impact how openpilot drives."), ""},
    {"DriverCamera", tr("Driver Camera On Reverse"), tr("Show the driver camera feed when in reverse."), ""},
    {"HideSpeed", tr("Hide Speed"), tr("Hide the speed indicator in the onroad UI. Additional toggle allows it to be hidden/shown via tapping the speed itself."), ""},
    {"MapStyle", tr("Map Style"), tr("Select a map style to use with navigation."), ""},
    {"StoppedTimer", tr("Stopped Timer"), tr("Display a timer in the onroad UI that indicates how long you've been stopped for."), ""},
    {"WheelSpeed", tr("Use Wheel Speed"), tr("Use the wheel speed instead of the cluster speed in the onroad UI."), ""},

    {"ScreenManagement", tr("Screen Management"), tr("Manage your screen's brightness, timeout settings, and hide onroad UI elements."), "../frogpilot/assets/toggle_icons/icon_light.png"},
    {"HideUIElements", tr("Hide UI Elements"), tr("Hide the selected UI elements from the onroad screen."), ""},
    {"ScreenBrightness", tr("Screen Brightness"), tr("Customize your screen brightness when offroad."), ""},
    {"ScreenBrightnessOnroad", tr("Screen Brightness (Onroad)"), tr("Customize your screen brightness when onroad."), ""},
    {"ScreenRecorder", tr("Screen Recorder"), tr("Enable the ability to record the screen while onroad."), ""},
    {"ScreenTimeout", tr("Screen Timeout"), tr("Customize how long it takes for your screen to turn off."), ""},
    {"ScreenTimeoutOnroad", tr("Screen Timeout (Onroad)"), tr("Customize how long it takes for your screen to turn off when onroad."), ""},
    {"StandbyMode", tr("Standby Mode"), tr("Turn the screen off after your screen times out when onroad, but wake it back up when engagement state changes or important alerts are triggered."), ""},
  };

  for (const auto &[param, title, desc, icon] : visualToggles) {
    AbstractControl *visualToggle;

    if (param == "BonusContent") {
      FrogPilotParamManageControl *BonusContentToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(BonusContentToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        showToggles(bonusContentKeys);
      });
      visualToggle = BonusContentToggle;
    } else if (param == "PersonalizeOpenpilot") {
      FrogPilotParamManageControl *personalizeOpenpilotToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(personalizeOpenpilotToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        personalizeOpenpilotOpen = true;
        openSubParentToggle();
        showToggles(personalizeOpenpilotKeys);
      });
      visualToggle = personalizeOpenpilotToggle;
    } else if (param == "CustomColors") {
      manageCustomColorsBtn = new FrogPilotButtonsControl(title, desc, {tr("DELETE"), tr("DOWNLOAD"), tr("SELECT")});

      std::function<QString(QString)> formatColorName = [](QString name) -> QString {
        QChar separator = name.contains('_') ? '_' : '-';
        QStringList parts = name.replace(separator, ' ').split(' ');

        for (int i = 0; i < parts.size(); ++i) {
          parts[i][0] = parts[i][0].toUpper();
        }

        if (separator == '-' && parts.size() > 1) {
          return parts.first() + " (" + parts.last() + ")";
        }

        return parts.join(' ');
      };

      std::function<QString(QString)> formatColorNameForStorage = [](QString name) -> QString {
        name = name.toLower();
        name = name.replace(" (", "-");
        name = name.replace(' ', '_');
        name.remove('(').remove(')');
        return name;
      };

      QObject::connect(manageCustomColorsBtn, &FrogPilotButtonsControl::buttonClicked, [=](int id) {
        QDir themesDir{"/data/themes/theme_packs"};
        QFileInfoList dirList = themesDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);

        QString currentColor = QString::fromStdString(params.get("CustomColors")).replace('_', ' ').replace('-', " (").toLower();
        currentColor[0] = currentColor[0].toUpper();
        for (int i = 1; i < currentColor.length(); i++) {
          if (currentColor[i - 1] == ' ' || currentColor[i - 1] == '(') {
            currentColor[i] = currentColor[i].toUpper();
          }
        }
        if (currentColor.contains(" (")) {
          currentColor.append(')');
        }

        QStringList availableColors;
        for (const QFileInfo &dirInfo : dirList) {
          QString colorSchemeDir = dirInfo.absoluteFilePath();

          if (QDir(colorSchemeDir + "/colors").exists()) {
            availableColors << formatColorName(dirInfo.fileName());
          }
        }
        availableColors.append("Stock");
        std::sort(availableColors.begin(), availableColors.end());

        if (id == 0) {
          QStringList colorSchemesList = availableColors;
          colorSchemesList.removeAll("Stock");
          colorSchemesList.removeAll(currentColor);

          QString colorSchemeToDelete = MultiOptionDialog::getSelection(tr("Select a color scheme to delete"), colorSchemesList, "", this);
          if (!colorSchemeToDelete.isEmpty() && ConfirmationDialog::confirm(tr("Are you sure you want to delete the '%1' color scheme?").arg(colorSchemeToDelete), tr("Delete"), this)) {
            themeDeleting = true;
            colorsDownloaded = false;

            QString selectedColor = formatColorNameForStorage(colorSchemeToDelete);
            for (const QFileInfo &dirInfo : dirList) {
              if (dirInfo.fileName() == selectedColor) {
                QDir colorDir(dirInfo.absoluteFilePath() + "/colors");
                if (colorDir.exists()) {
                  colorDir.removeRecursively();
                }
              }
            }

            QStringList downloadableColors = QString::fromStdString(params.get("DownloadableColors")).split(",");
            downloadableColors << colorSchemeToDelete;
            downloadableColors.removeDuplicates();
            downloadableColors.removeAll("");
            std::sort(downloadableColors.begin(), downloadableColors.end());

            params.put("DownloadableColors", downloadableColors.join(",").toStdString());
            themeDeleting = false;
          }
        } else if (id == 1) {
          if (colorDownloading) {
            paramsMemory.putBool("CancelThemeDownload", true);
            cancellingDownload = true;

            QTimer::singleShot(2000, [=]() {
              paramsMemory.putBool("CancelThemeDownload", false);
              cancellingDownload = false;
              colorDownloading = false;
              themeDownloading = false;

              device()->resetInteractiveTimeout(30);
            });
          } else {
            QStringList downloadableColors = QString::fromStdString(params.get("DownloadableColors")).split(",");
            QString colorSchemeToDownload = MultiOptionDialog::getSelection(tr("Select a color scheme to download"), downloadableColors, "", this);

            if (!colorSchemeToDownload.isEmpty()) {
              device()->resetInteractiveTimeout(300);

              QString convertedColorScheme = formatColorNameForStorage(colorSchemeToDownload);
              paramsMemory.put("ColorToDownload", convertedColorScheme.toStdString());
              downloadStatusLabel->setText("Downloading...");
              paramsMemory.put("ThemeDownloadProgress", "Downloading...");
              colorDownloading = true;
              themeDownloading = true;

              downloadableColors.removeAll(colorSchemeToDownload);
              params.put("DownloadableColors", downloadableColors.join(",").toStdString());
            }
          }
        } else if (id == 2) {
          QString colorSchemeToSelect = MultiOptionDialog::getSelection(tr("Select a color scheme"), availableColors, currentColor, this);
          if (!colorSchemeToSelect.isEmpty()) {
            params.put("CustomColors", formatColorNameForStorage(colorSchemeToSelect).toStdString());
            loadThemeColors("", true);
            manageCustomColorsBtn->setValue(colorSchemeToSelect);
            paramsMemory.putBool("UpdateTheme", true);
          }
        }
      });

      QString currentColor = QString::fromStdString(params.get("CustomColors")).replace('_', ' ').replace('-', " (").toLower();
      currentColor[0] = currentColor[0].toUpper();
      for (int i = 1; i < currentColor.length(); i++) {
        if (currentColor[i - 1] == ' ' || currentColor[i - 1] == '(') {
          currentColor[i] = currentColor[i].toUpper();
        }
      }
      if (currentColor.contains(" (")) {
        currentColor.append(')');
      }
      manageCustomColorsBtn->setValue(currentColor);
      visualToggle = manageCustomColorsBtn;
    } else if (param == "CustomIcons") {
      manageCustomIconsBtn = new FrogPilotButtonsControl(title, desc, {tr("DELETE"), tr("DOWNLOAD"), tr("SELECT")});

      std::function<QString(QString)> formatIconName = [](QString name) -> QString {
        QChar separator = name.contains('_') ? '_' : '-';
        QStringList parts = name.replace(separator, ' ').split(' ');

        for (int i = 0; i < parts.size(); ++i) {
          parts[i][0] = parts[i][0].toUpper();
        }

        if (separator == '-' && parts.size() > 1) {
          return parts.first() + " (" + parts.last() + ")";
        }

        return parts.join(' ');
      };

      std::function<QString(QString)> formatIconNameForStorage = [](QString name) -> QString {
        name = name.toLower();
        name = name.replace(" (", "-");
        name = name.replace(' ', '_');
        name.remove('(').remove(')');
        return name;
      };

      QObject::connect(manageCustomIconsBtn, &FrogPilotButtonsControl::buttonClicked, [=](int id) {
        QDir themesDir{"/data/themes/theme_packs"};
        QFileInfoList dirList = themesDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);

        QString currentIcon = QString::fromStdString(params.get("CustomIcons")).replace('_', ' ').replace('-', " (").toLower();
        currentIcon[0] = currentIcon[0].toUpper();
        for (int i = 1; i < currentIcon.length(); i++) {
          if (currentIcon[i - 1] == ' ' || currentIcon[i - 1] == '(') {
            currentIcon[i] = currentIcon[i].toUpper();
          }
        }
        if (currentIcon.contains(" (")) {
          currentIcon.append(')');
        }

        QStringList availableIcons;
        for (const QFileInfo &dirInfo : dirList) {
          QString iconDir = dirInfo.absoluteFilePath();
          if (QDir(iconDir + "/icons").exists()) {
            availableIcons << formatIconName(dirInfo.fileName());
          }
        }
        availableIcons.append("Stock");
        std::sort(availableIcons.begin(), availableIcons.end());

        if (id == 0) {
          QStringList customIconsList = availableIcons;
          customIconsList.removeAll("Stock");
          customIconsList.removeAll(currentIcon);

          QString iconPackToDelete = MultiOptionDialog::getSelection(tr("Select an icon pack to delete"), customIconsList, "", this);
          if (!iconPackToDelete.isEmpty() && ConfirmationDialog::confirm(tr("Are you sure you want to delete the '%1' icon pack?").arg(iconPackToDelete), tr("Delete"), this)) {
            themeDeleting = true;
            iconsDownloaded = false;

            QString selectedIcon = formatIconNameForStorage(iconPackToDelete);
            for (const QFileInfo &dirInfo : dirList) {
              if (dirInfo.fileName() == selectedIcon) {
                QDir iconDir(dirInfo.absoluteFilePath() + "/icons");
                if (iconDir.exists()) {
                  iconDir.removeRecursively();
                }
              }
            }

            QStringList downloadableIcons = QString::fromStdString(params.get("DownloadableIcons")).split(",");
            downloadableIcons << iconPackToDelete;
            downloadableIcons.removeDuplicates();
            downloadableIcons.removeAll("");
            std::sort(downloadableIcons.begin(), downloadableIcons.end());

            params.put("DownloadableIcons", downloadableIcons.join(",").toStdString());
            themeDeleting = false;
          }
        } else if (id == 1) {
          if (iconDownloading) {
            paramsMemory.putBool("CancelThemeDownload", true);
            cancellingDownload = true;

            QTimer::singleShot(2000, [=]() {
              paramsMemory.putBool("CancelThemeDownload", false);
              cancellingDownload = false;
              iconDownloading = false;
              themeDownloading = false;

              device()->resetInteractiveTimeout(30);
            });
          } else {
            QStringList downloadableIcons = QString::fromStdString(params.get("DownloadableIcons")).split(",");
            QString iconPackToDownload = MultiOptionDialog::getSelection(tr("Select an icon pack to download"), downloadableIcons, "", this);

            if (!iconPackToDownload.isEmpty()) {
              device()->resetInteractiveTimeout(300);

              QString convertedIconPack = formatIconNameForStorage(iconPackToDownload);
              paramsMemory.put("IconToDownload", convertedIconPack.toStdString());
              downloadStatusLabel->setText("Downloading...");
              paramsMemory.put("ThemeDownloadProgress", "Downloading...");
              iconDownloading = true;
              themeDownloading = true;

              downloadableIcons.removeAll(iconPackToDownload);
              params.put("DownloadableIcons", downloadableIcons.join(",").toStdString());
            }
          }
        } else if (id == 2) {
          QString iconPackToSelect = MultiOptionDialog::getSelection(tr("Select an icon pack"), availableIcons, currentIcon, this);
          if (!iconPackToSelect.isEmpty()) {
            params.put("CustomIcons", formatIconNameForStorage(iconPackToSelect).toStdString());
            manageCustomIconsBtn->setValue(iconPackToSelect);
            paramsMemory.putBool("UpdateTheme", true);
          }
        }
      });

      QString currentIcon = QString::fromStdString(params.get("CustomIcons")).replace('_', ' ').replace('-', " (").toLower();
      currentIcon[0] = currentIcon[0].toUpper();
      for (int i = 1; i < currentIcon.length(); i++) {
        if (currentIcon[i - 1] == ' ' || currentIcon[i - 1] == '(') {
          currentIcon[i] = currentIcon[i].toUpper();
        }
      }
      if (currentIcon.contains(" (")) {
        currentIcon.append(')');
      }
      manageCustomIconsBtn->setValue(currentIcon);
      visualToggle = manageCustomIconsBtn;
    } else if (param == "CustomSignals") {
      manageCustomSignalsBtn = new FrogPilotButtonsControl(title, desc, {tr("DELETE"), tr("DOWNLOAD"), tr("SELECT")});

      std::function<QString(QString)> formatSignalName = [](QString name) -> QString {
        QChar separator = name.contains('_') ? '_' : '-';
        QStringList parts = name.replace(separator, ' ').split(' ');

        for (int i = 0; i < parts.size(); ++i) {
          parts[i][0] = parts[i][0].toUpper();
        }

        if (separator == '-' && parts.size() > 1) {
          return parts.first() + " (" + parts.last() + ")";
        }

        return parts.join(' ');
      };

      std::function<QString(QString)> formatSignalNameForStorage = [](QString name) -> QString {
        name = name.toLower();
        name = name.replace(" (", "-");
        name = name.replace(' ', '_');
        name.remove('(').remove(')');
        return name;
      };

      QObject::connect(manageCustomSignalsBtn, &FrogPilotButtonsControl::buttonClicked, [=](int id) {
        QDir themesDir{"/data/themes/theme_packs"};
        QFileInfoList dirList = themesDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);

        QString currentSignal = QString::fromStdString(params.get("CustomSignals")).replace('_', ' ').replace('-', " (").toLower();
        currentSignal[0] = currentSignal[0].toUpper();
        for (int i = 1; i < currentSignal.length(); i++) {
          if (currentSignal[i - 1] == ' ' || currentSignal[i - 1] == '(') {
            currentSignal[i] = currentSignal[i].toUpper();
          }
        }
        if (currentSignal.contains(" (")) {
          currentSignal.append(')');
        }

        QStringList availableSignals;
        for (const QFileInfo &dirInfo : dirList) {
          QString signalDir = dirInfo.absoluteFilePath();
          if (QDir(signalDir + "/signals").exists()) {
            availableSignals << formatSignalName(dirInfo.fileName());
          }
        }
        availableSignals.append("Stock");
        std::sort(availableSignals.begin(), availableSignals.end());

        if (id == 0) {
          QStringList customSignalsList = availableSignals;
          customSignalsList.removeAll("Stock");
          customSignalsList.removeAll(currentSignal);

          QString signalPackToDelete = MultiOptionDialog::getSelection(tr("Select a signal pack to delete"), customSignalsList, "", this);
          if (!signalPackToDelete.isEmpty() && ConfirmationDialog::confirm(tr("Are you sure you want to delete the '%1' signal pack?").arg(signalPackToDelete), tr("Delete"), this)) {
            themeDeleting = true;
            signalsDownloaded = false;

            QString selectedSignal = formatSignalNameForStorage(signalPackToDelete);
            for (const QFileInfo &dirInfo : dirList) {
              if (dirInfo.fileName() == selectedSignal) {
                QDir signalDir(dirInfo.absoluteFilePath() + "/signals");
                if (signalDir.exists()) {
                  signalDir.removeRecursively();
                }
              }
            }

            QStringList downloadableSignals = QString::fromStdString(params.get("DownloadableSignals")).split(",");
            downloadableSignals << signalPackToDelete;
            downloadableSignals.removeDuplicates();
            downloadableSignals.removeAll("");
            std::sort(downloadableSignals.begin(), downloadableSignals.end());

            params.put("DownloadableSignals", downloadableSignals.join(",").toStdString());
            themeDeleting = false;
          }
        } else if (id == 1) {
          if (signalDownloading) {
            paramsMemory.putBool("CancelThemeDownload", true);
            cancellingDownload = true;

            QTimer::singleShot(2000, [=]() {
              paramsMemory.putBool("CancelThemeDownload", false);
              cancellingDownload = false;
              signalDownloading = false;
              themeDownloading = false;

              device()->resetInteractiveTimeout(30);
            });
          } else {
            QStringList downloadableSignals = QString::fromStdString(params.get("DownloadableSignals")).split(",");
            QString signalPackToDownload = MultiOptionDialog::getSelection(tr("Select a signal pack to download"), downloadableSignals, "", this);

            if (!signalPackToDownload.isEmpty()) {
              device()->resetInteractiveTimeout(300);

              QString convertedSignalPack = formatSignalNameForStorage(signalPackToDownload);
              paramsMemory.put("SignalToDownload", convertedSignalPack.toStdString());
              downloadStatusLabel->setText("Downloading...");
              paramsMemory.put("ThemeDownloadProgress", "Downloading...");
              signalDownloading = true;
              themeDownloading = true;

              downloadableSignals.removeAll(signalPackToDownload);
              params.put("DownloadableSignals", downloadableSignals.join(",").toStdString());
            }
          }
        } else if (id == 2) {
          QString signalPackToSelect = MultiOptionDialog::getSelection(tr("Select a signal pack"), availableSignals, currentSignal, this);
          if (!signalPackToSelect.isEmpty()) {
            params.put("CustomSignals", formatSignalNameForStorage(signalPackToSelect).toStdString());
            manageCustomSignalsBtn->setValue(signalPackToSelect);
            paramsMemory.putBool("UpdateTheme", true);
          }
        }
      });

      QString currentSignal = QString::fromStdString(params.get("CustomSignals")).replace('_', ' ').replace('-', " (").toLower();
      currentSignal[0] = currentSignal[0].toUpper();
      for (int i = 1; i < currentSignal.length(); i++) {
        if (currentSignal[i - 1] == ' ' || currentSignal[i - 1] == '(') {
          currentSignal[i] = currentSignal[i].toUpper();
        }
      }
      if (currentSignal.contains(" (")) {
        currentSignal.append(')');
      }
      manageCustomSignalsBtn->setValue(currentSignal);
      visualToggle = manageCustomSignalsBtn;
    } else if (param == "CustomSounds") {
      manageCustomSoundsBtn = new FrogPilotButtonsControl(title, desc, {tr("DELETE"), tr("DOWNLOAD"), tr("SELECT")});

      std::function<QString(QString)> formatSoundName = [](QString name) -> QString {
        QChar separator = name.contains('_') ? '_' : '-';
        QStringList parts = name.replace(separator, ' ').split(' ');

        for (int i = 0; i < parts.size(); ++i) {
          parts[i][0] = parts[i][0].toUpper();
        }

        if (separator == '-' && parts.size() > 1) {
          return parts.first() + " (" + parts.last() + ")";
        }

        return parts.join(' ');
      };

      std::function<QString(QString)> formatSoundNameForStorage = [](QString name) -> QString {
        name = name.toLower();
        name = name.replace(" (", "-");
        name = name.replace(' ', '_');
        name.remove('(').remove(')');
        return name;
      };

      QObject::connect(manageCustomSoundsBtn, &FrogPilotButtonsControl::buttonClicked, [=](int id) {
        QDir themesDir{"/data/themes/theme_packs"};
        QFileInfoList dirList = themesDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);

        QString currentSound = QString::fromStdString(params.get("CustomSounds")).replace('_', ' ').replace('-', " (").toLower();
        currentSound[0] = currentSound[0].toUpper();
        for (int i = 1; i < currentSound.length(); i++) {
          if (currentSound[i - 1] == ' ' || currentSound[i - 1] == '(') {
            currentSound[i] = currentSound[i].toUpper();
          }
        }
        if (currentSound.contains(" (")) {
          currentSound.append(')');
        }

        QStringList availableSounds;
        for (const QFileInfo &dirInfo : dirList) {
          QString soundDir = dirInfo.absoluteFilePath();
          if (QDir(soundDir + "/sounds").exists()) {
            availableSounds << formatSoundName(dirInfo.fileName());
          }
        }
        availableSounds.append("Stock");
        std::sort(availableSounds.begin(), availableSounds.end());

        if (id == 0) {
          QStringList customSoundsList = availableSounds;
          customSoundsList.removeAll("Stock");
          customSoundsList.removeAll(currentSound);

          QString soundSchemeToDelete = MultiOptionDialog::getSelection(tr("Select a sound pack to delete"), customSoundsList, "", this);
          if (!soundSchemeToDelete.isEmpty() && ConfirmationDialog::confirm(tr("Are you sure you want to delete the '%1' sound scheme?").arg(soundSchemeToDelete), tr("Delete"), this)) {
            themeDeleting = true;
            soundsDownloaded = false;

            QString selectedSound = formatSoundNameForStorage(soundSchemeToDelete);
            for (const QFileInfo &dirInfo : dirList) {
              if (dirInfo.fileName() == selectedSound) {
                QDir soundDir(dirInfo.absoluteFilePath() + "/sounds");
                if (soundDir.exists()) {
                  soundDir.removeRecursively();
                }
              }
            }

            QStringList downloadableSounds = QString::fromStdString(params.get("DownloadableSounds")).split(",");
            downloadableSounds << soundSchemeToDelete;
            downloadableSounds.removeDuplicates();
            downloadableSounds.removeAll("");
            std::sort(downloadableSounds.begin(), downloadableSounds.end());

            params.put("DownloadableSounds", downloadableSounds.join(",").toStdString());
            themeDeleting = false;
          }
        } else if (id == 1) {
          if (soundDownloading) {
            paramsMemory.putBool("CancelThemeDownload", true);
            cancellingDownload = true;

            QTimer::singleShot(2000, [=]() {
              paramsMemory.putBool("CancelThemeDownload", false);
              cancellingDownload = false;
              soundDownloading = false;
              themeDownloading = false;

              device()->resetInteractiveTimeout(30);
            });
          } else {
            QStringList downloadableSounds = QString::fromStdString(params.get("DownloadableSounds")).split(",");
            QString soundSchemeToDownload = MultiOptionDialog::getSelection(tr("Select a sound pack to download"), downloadableSounds, "", this);

            if (!soundSchemeToDownload.isEmpty()) {
              device()->resetInteractiveTimeout(300);

              QString convertedSoundScheme = formatSoundNameForStorage(soundSchemeToDownload);
              paramsMemory.put("SoundToDownload", convertedSoundScheme.toStdString());
              downloadStatusLabel->setText("Downloading...");
              paramsMemory.put("ThemeDownloadProgress", "Downloading...");
              soundDownloading = true;
              themeDownloading = true;

              downloadableSounds.removeAll(soundSchemeToDownload);
              params.put("DownloadableSounds", downloadableSounds.join(",").toStdString());
            }
          }
        } else if (id == 2) {
          QString soundSchemeToSelect = MultiOptionDialog::getSelection(tr("Select a sound scheme"), availableSounds, currentSound, this);
          if (!soundSchemeToSelect.isEmpty()) {
            params.put("CustomSounds", formatSoundNameForStorage(soundSchemeToSelect).toStdString());
            manageCustomSoundsBtn->setValue(soundSchemeToSelect);
            paramsMemory.putBool("UpdateTheme", true);
          }
        }
      });

      QString currentSound = QString::fromStdString(params.get("CustomSounds")).replace('_', ' ').replace('-', " (").toLower();
      currentSound[0] = currentSound[0].toUpper();
      for (int i = 1; i < currentSound.length(); i++) {
        if (currentSound[i - 1] == ' ' || currentSound[i - 1] == '(') {
          currentSound[i] = currentSound[i].toUpper();
        }
      }
      if (currentSound.contains(" (")) {
        currentSound.append(')');
      }
      manageCustomSoundsBtn->setValue(currentSound);
      visualToggle = manageCustomSoundsBtn;
    } else if (param == "WheelIcon") {
      manageWheelIconsBtn = new FrogPilotButtonsControl(title, desc, {tr("DELETE"), tr("DOWNLOAD"), tr("SELECT")});

      std::function<QString(QString)> formatWheelName = [](QString name) -> QString {
        QChar separator = name.contains('_') ? '_' : '-';
        QStringList parts = name.replace(separator, ' ').split(' ');

        for (int i = 0; i < parts.size(); ++i) {
          parts[i][0] = parts[i][0].toUpper();
        }

        if (separator == '-' && parts.size() > 1) {
          return parts.first() + " (" + parts.last() + ")";
        }

        return parts.join(' ');
      };

      std::function<QString(QString)> formatWheelNameForStorage = [](QString name) -> QString {
        name = name.toLower();
        name = name.replace(" (", "-");
        name = name.replace(' ', '_');
        name.remove('(').remove(')');
        return name;
      };

      QObject::connect(manageWheelIconsBtn, &FrogPilotButtonsControl::buttonClicked, [=](int id) {
        QDir wheelDir{"/data/themes/steering_wheels"};
        QFileInfoList fileList = wheelDir.entryInfoList(QDir::Files);

        QString currentWheel = QString::fromStdString(params.get("WheelIcon")).replace('_', ' ').replace('-', " (").toLower();
        currentWheel[0] = currentWheel[0].toUpper();
        for (int i = 1; i < currentWheel.length(); i++) {
          if (currentWheel[i - 1] == ' ' || currentWheel[i - 1] == '(') {
            currentWheel[i] = currentWheel[i].toUpper();
          }
        }
        if (currentWheel.contains(" (")) {
          currentWheel.append(')');
        }

        QStringList availableWheels;
        for (const QFileInfo &fileInfo : fileList) {
          QString baseName = fileInfo.completeBaseName();
          QString formattedName = formatWheelName(baseName);
          if (formattedName != "Img Chffr Wheel") {
            availableWheels << formattedName;
          }
        }
        availableWheels.append("Stock");
        availableWheels.append("None");
        std::sort(availableWheels.begin(), availableWheels.end());

        if (id == 0) {
          QStringList steeringWheelList = availableWheels;
          steeringWheelList.removeAll("None");
          steeringWheelList.removeAll("Stock");
          steeringWheelList.removeAll(currentWheel);

          QString imageToDelete = MultiOptionDialog::getSelection(tr("Select a steering wheel to delete"), steeringWheelList, "", this);
          if (!imageToDelete.isEmpty() && ConfirmationDialog::confirm(tr("Are you sure you want to delete the '%1' steering wheel image?").arg(imageToDelete), tr("Delete"), this)) {
            themeDeleting = true;
            wheelsDownloaded = false;

            QString selectedImage = formatWheelNameForStorage(imageToDelete);
            for (const QFileInfo &fileInfo : fileList) {
              if (fileInfo.completeBaseName() == selectedImage) {
                QFile::remove(fileInfo.filePath());
              }
            }

            QStringList downloadableWheels = QString::fromStdString(params.get("DownloadableWheels")).split(",");
            downloadableWheels << imageToDelete;
            downloadableWheels.removeDuplicates();
            downloadableWheels.removeAll("");
            std::sort(downloadableWheels.begin(), downloadableWheels.end());

            params.put("DownloadableWheels", downloadableWheels.join(",").toStdString());
            themeDeleting = false;
          }
        } else if (id == 1) {
          if (wheelDownloading) {
            paramsMemory.putBool("CancelThemeDownload", true);
            cancellingDownload = true;

            QTimer::singleShot(2000, [=]() {
              paramsMemory.putBool("CancelThemeDownload", false);
              cancellingDownload = false;
              wheelDownloading = false;
              themeDownloading = false;

              device()->resetInteractiveTimeout(30);
            });
          } else {
            QStringList downloadableWheels = QString::fromStdString(params.get("DownloadableWheels")).split(",");
            QString wheelToDownload = MultiOptionDialog::getSelection(tr("Select a steering wheel to download"), downloadableWheels, "", this);

            if (!wheelToDownload.isEmpty()) {
              device()->resetInteractiveTimeout(300);

              QString convertedImage = formatWheelNameForStorage(wheelToDownload);
              paramsMemory.put("WheelToDownload", convertedImage.toStdString());
              downloadStatusLabel->setText("Downloading...");
              paramsMemory.put("ThemeDownloadProgress", "Downloading...");
              themeDownloading = true;
              wheelDownloading = true;

              downloadableWheels.removeAll(wheelToDownload);
              params.put("DownloadableWheels", downloadableWheels.join(",").toStdString());
            }
          }
        } else if (id == 2) {
          QString imageToSelect = MultiOptionDialog::getSelection(tr("Select a steering wheel"), availableWheels, currentWheel, this);
          if (!imageToSelect.isEmpty()) {
            params.put("WheelIcon", formatWheelNameForStorage(imageToSelect).toStdString());
            manageWheelIconsBtn->setValue(imageToSelect);
            paramsMemory.putBool("UpdateTheme", true);
          }
        }
      });

      QString currentWheel = QString::fromStdString(params.get("WheelIcon")).replace('_', ' ').replace('-', " (").toLower();
      currentWheel[0] = currentWheel[0].toUpper();
      for (int i = 1; i < currentWheel.length(); i++) {
        if (currentWheel[i - 1] == ' ' || currentWheel[i - 1] == '(') {
          currentWheel[i] = currentWheel[i].toUpper();
        }
      }
      if (currentWheel.contains(" (")) {
        currentWheel.append(')');
      }
      manageWheelIconsBtn->setValue(currentWheel);
      visualToggle = manageWheelIconsBtn;
    } else if (param == "DownloadStatusLabel") {
      downloadStatusLabel = new LabelControl(title, "Idle");
      visualToggle = reinterpret_cast<AbstractControl*>(downloadStatusLabel);
    } else if (param == "StartupAlert") {
      FrogPilotButtonsControl *startupAlertButton = new FrogPilotButtonsControl(title, desc, {tr("STOCK"), tr("FROGPILOT"), tr("CUSTOM"), tr("CLEAR")});
      QObject::connect(startupAlertButton, &FrogPilotButtonsControl::buttonClicked, [=](int id) {
        int maxLengthTop = 35;
        int maxLengthBottom = 45;

        QString stockTop = "Be ready to take over at any time";
        QString stockBottom = "Always keep hands on wheel and eyes on road";

        QString frogpilotTop = "Hippity hoppity this is my property";
        QString frogpilotBottom = "so I do what I want 🐸";

        QString currentTop = QString::fromStdString(params.get("StartupMessageTop"));
        QString currentBottom = QString::fromStdString(params.get("StartupMessageBottom"));

        if (id == 0) {
          params.put("StartupMessageTop", stockTop.toStdString());
          params.put("StartupMessageBottom", stockBottom.toStdString());
        } else if (id == 1) {
          params.put("StartupMessageTop", frogpilotTop.toStdString());
          params.put("StartupMessageBottom", frogpilotBottom.toStdString());
        } else if (id == 2) {
          QString newTop = InputDialog::getText(tr("Enter your text for the top half"), this, tr("Characters: 0/%1").arg(maxLengthTop), false, -1, currentTop, maxLengthTop).trimmed();
          if (newTop.length() > 0) {
            params.putNonBlocking("StartupMessageTop", newTop.toStdString());
            QString newBottom = InputDialog::getText(tr("Enter your text for the bottom half"), this, tr("Characters: 0/%1").arg(maxLengthBottom), false, -1, currentBottom, maxLengthBottom).trimmed();
            if (newBottom.length() > 0) {
              params.putNonBlocking("StartupMessageBottom", newBottom.toStdString());
            }
          }
        } else if (id == 3) {
          params.remove("StartupMessageTop");
          params.remove("StartupMessageBottom");
        }
      });
      visualToggle = startupAlertButton;

    } else if (param == "CustomUI") {
      FrogPilotParamManageControl *customUIToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(customUIToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        showToggles(customOnroadUIKeys);
      });
      visualToggle = customUIToggle;
    } else if (param == "CustomPaths") {
      std::vector<QString> pathToggles{"AccelerationPath", "AdjacentPath", "BlindSpotPath", "AdjacentPathMetrics"};
      std::vector<QString> pathToggleNames{tr("Acceleration"), tr("Adjacent"), tr("Blind Spot"), tr("Metrics")};
      visualToggle = new FrogPilotButtonToggleControl(param, title, desc, pathToggles, pathToggleNames);
    } else if (param == "PedalsOnUI") {
      std::vector<QString> pedalsToggles{"DynamicPedalsOnUI", "StaticPedalsOnUI"};
      std::vector<QString> pedalsToggleNames{tr("Dynamic"), tr("Static")};
      FrogPilotButtonToggleControl *pedalsToggle = new FrogPilotButtonToggleControl(param, title, desc, pedalsToggles, pedalsToggleNames, true);
      QObject::connect(pedalsToggle, &FrogPilotButtonToggleControl::buttonClicked, [this](int index) {
        if (index == 0) {
          params.putBool("StaticPedalsOnUI", false);
        } else if (index == 1) {
          params.putBool("DynamicPedalsOnUI", false);
        }
      });
      visualToggle = pedalsToggle;
    } else if (param == "ShowStoppingPoint") {
      std::vector<QString> stoppingPointToggles{"ShowStoppingPointMetrics"};
      std::vector<QString> stoppingPointToggleNames{tr("Show Distance")};
      visualToggle = new FrogPilotButtonToggleControl(param, title, desc, stoppingPointToggles, stoppingPointToggleNames);

    } else if (param == "QOLVisuals") {
      FrogPilotParamManageControl *qolToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(qolToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        showToggles(qolKeys);
      });
      visualToggle = qolToggle;
    } else if (param == "CameraView") {
      std::vector<QString> cameraOptions{tr("Auto"), tr("Driver"), tr("Standard"), tr("Wide")};
      ButtonParamControl *preferredCamera = new ButtonParamControl(param, title, desc, icon, cameraOptions);
      visualToggle = preferredCamera;
    } else if (param == "BigMap") {
      std::vector<QString> mapToggles{"FullMap"};
      std::vector<QString> mapToggleNames{tr("Full Map")};
      visualToggle = new FrogPilotButtonToggleControl(param, title, desc, mapToggles, mapToggleNames);
    } else if (param == "HideSpeed") {
      std::vector<QString> hideSpeedToggles{"HideSpeedUI"};
      std::vector<QString> hideSpeedToggleNames{tr("Control Via UI")};
      visualToggle = new FrogPilotButtonToggleControl(param, title, desc, hideSpeedToggles, hideSpeedToggleNames);
    } else if (param == "MapStyle") {
      QMap<int, QString> styleMap = {
        {0, tr("Stock openpilot")},
        {1, tr("Mapbox Streets")},
        {2, tr("Mapbox Outdoors")},
        {3, tr("Mapbox Light")},
        {4, tr("Mapbox Dark")},
        {5, tr("Mapbox Satellite")},
        {6, tr("Mapbox Satellite Streets")},
        {7, tr("Mapbox Navigation Day")},
        {8, tr("Mapbox Navigation Night")},
        {9, tr("Mapbox Traffic Night")},
        {10, tr("mike854's (Satellite hybrid)")},
      };

      QStringList styles = styleMap.values();
      ButtonControl *mapStyleButton = new ButtonControl(title, tr("SELECT"), desc);
      QObject::connect(mapStyleButton, &ButtonControl::clicked, [=]() {
        QStringList styles = styleMap.values();
        QString selection = MultiOptionDialog::getSelection(tr("Select a map style"), styles, "", this);
        if (!selection.isEmpty()) {
          int selectedStyle = styleMap.key(selection);
          params.putIntNonBlocking("MapStyle", selectedStyle);
          mapStyleButton->setValue(selection);
          updateFrogPilotToggles();
        }
      });

      int currentStyle = params.getInt("MapStyle");
      mapStyleButton->setValue(styleMap[currentStyle]);

      visualToggle = mapStyleButton;

    } else if (param == "ScreenManagement") {
      FrogPilotParamManageControl *screenToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(screenToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        showToggles(screenKeys);
      });
      visualToggle = screenToggle;
    } else if (param == "HideUIElements") {
      std::vector<QString> uiElementsToggles{"HideAlerts", "HideMapIcon", "HideMaxSpeed"};
      std::vector<QString> uiElementsToggleNames{tr("Alerts"), tr("Map Icon"), tr("Max Speed")};
      visualToggle = new FrogPilotButtonToggleControl(param, title, desc, uiElementsToggles, uiElementsToggleNames);
    } else if (param == "ScreenBrightness" || param == "ScreenBrightnessOnroad") {
      std::map<int, QString> brightnessLabels;
      int minBrightness = (param == "ScreenBrightnessOnroad") ? 0 : 1;
      for (int i = 1; i <= 101; i++) {
        brightnessLabels[i] = (i == 101) ? tr("Auto") : QString::number(i) + "%";
      }
      visualToggle = new FrogPilotParamValueControl(param, title, desc, icon, minBrightness, 101, QString(), brightnessLabels, 1, false, true);
    } else if (param == "ScreenTimeout" || param == "ScreenTimeoutOnroad") {
      visualToggle = new FrogPilotParamValueControl(param, title, desc, icon, 5, 60, tr(" seconds"));

    } else {
      visualToggle = new ParamControl(param, title, desc, icon);
    }

    addItem(visualToggle);
    toggles[param] = visualToggle;

    tryConnect(visualToggle, &updateFrogPilotToggles);

    if (FrogPilotParamManageControl *frogPilotManageToggle = qobject_cast<FrogPilotParamManageControl*>(visualToggle)) {
      QObject::connect(frogPilotManageToggle, &FrogPilotParamManageControl::manageButtonClicked, this, &FrogPilotVisualsPanel::openParentToggle);
    }

    QObject::connect(visualToggle, &AbstractControl::showDescriptionEvent, [this]() {
      update();
    });
  }

  FrogPilotParamValueControl *screenBrightnessToggle = static_cast<FrogPilotParamValueControl*>(toggles["ScreenBrightness"]);
  QObject::connect(screenBrightnessToggle, &FrogPilotParamValueControl::valueChanged, [this](float value) {
    if (!started) {
      uiState()->scene.screen_brightness = value;
    }
  });

  FrogPilotParamValueControl *screenBrightnessOnroadToggle = static_cast<FrogPilotParamValueControl*>(toggles["ScreenBrightnessOnroad"]);
  QObject::connect(screenBrightnessOnroadToggle, &FrogPilotParamValueControl::valueChanged, [this](float value) {
    if (started) {
      uiState()->scene.screen_brightness_onroad = value;
    }
  });

  QObject::connect(parent, &FrogPilotSettingsWindow::closeParentToggle, this, &FrogPilotVisualsPanel::hideToggles);
  QObject::connect(parent, &FrogPilotSettingsWindow::closeSubParentToggle, this, &FrogPilotVisualsPanel::hideSubToggles);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &FrogPilotVisualsPanel::updateCarToggles);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotVisualsPanel::updateState);

  hideToggles();
}

void FrogPilotVisualsPanel::showEvent(QShowEvent *event) {
  disableOpenpilotLongitudinal = params.getBool("DisableOpenpilotLongitudinal");

  colorsDownloaded = params.get("DownloadableColors").empty();
  iconsDownloaded = params.get("DownloadableIcons").empty();
  signalsDownloaded = params.get("DownloadableSignals").empty();
  soundsDownloaded = params.get("DownloadableSounds").empty();
  wheelsDownloaded = params.get("DownloadableWheels").empty();
}

void FrogPilotVisualsPanel::updateState(const UIState &s) {
  if (!isVisible()) return;

  if (personalizeOpenpilotOpen) {
    if (themeDownloading) {
      QString progress = QString::fromStdString(paramsMemory.get("ThemeDownloadProgress"));
      bool downloadFailed = progress.contains(QRegularExpression("cancelled|exists|Failed|offline", QRegularExpression::CaseInsensitiveOption));

      if (progress != "Downloading...") {
        downloadStatusLabel->setText(progress);
      }

      if (progress == "Downloaded!" || downloadFailed) {
        QTimer::singleShot(2000, [=]() {
          if (!themeDownloading) {
            downloadStatusLabel->setText("Idle");
          }

          device()->resetInteractiveTimeout(30);
        });
        paramsMemory.remove("ThemeDownloadProgress");
        colorDownloading = false;
        iconDownloading = false;
        signalDownloading = false;
        soundDownloading = false;
        themeDownloading = false;
        wheelDownloading = false;

        colorsDownloaded = params.get("DownloadableColors").empty();
        iconsDownloaded = params.get("DownloadableIcons").empty();
        signalsDownloaded = params.get("DownloadableSignals").empty();
        soundsDownloaded = params.get("DownloadableSounds").empty();
        wheelsDownloaded = params.get("DownloadableWheels").empty();
      }
    }

    manageCustomColorsBtn->setText(1, colorDownloading ? tr("CANCEL") : tr("DOWNLOAD"));
    manageCustomColorsBtn->setEnabledButtons(0, !themeDeleting && !themeDownloading);
    manageCustomColorsBtn->setEnabledButtons(1, s.scene.online && (!themeDownloading || colorDownloading) && !cancellingDownload && !themeDeleting && !colorsDownloaded);
    manageCustomColorsBtn->setEnabledButtons(2, !themeDeleting && !themeDownloading);

    manageCustomIconsBtn->setText(1, iconDownloading ? tr("CANCEL") : tr("DOWNLOAD"));
    manageCustomIconsBtn->setEnabledButtons(0, !themeDeleting && !themeDownloading);
    manageCustomIconsBtn->setEnabledButtons(1, s.scene.online && (!themeDownloading || iconDownloading) && !cancellingDownload && !themeDeleting && !iconsDownloaded);
    manageCustomIconsBtn->setEnabledButtons(2, !themeDeleting && !themeDownloading);

    manageCustomSignalsBtn->setText(1, signalDownloading ? tr("CANCEL") : tr("DOWNLOAD"));
    manageCustomSignalsBtn->setEnabledButtons(0, !themeDeleting && !themeDownloading);
    manageCustomSignalsBtn->setEnabledButtons(1, s.scene.online && (!themeDownloading || signalDownloading) && !cancellingDownload && !themeDeleting && !signalsDownloaded);
    manageCustomSignalsBtn->setEnabledButtons(2, !themeDeleting && !themeDownloading);

    manageCustomSoundsBtn->setText(1, soundDownloading ? tr("CANCEL") : tr("DOWNLOAD"));
    manageCustomSoundsBtn->setEnabledButtons(0, !themeDeleting && !themeDownloading);
    manageCustomSoundsBtn->setEnabledButtons(1, s.scene.online && (!themeDownloading || soundDownloading) && !cancellingDownload && !themeDeleting && !soundsDownloaded);
    manageCustomSoundsBtn->setEnabledButtons(2, !themeDeleting && !themeDownloading);

    manageWheelIconsBtn->setText(1, wheelDownloading ? tr("CANCEL") : tr("DOWNLOAD"));
    manageWheelIconsBtn->setEnabledButtons(0, !themeDeleting && !themeDownloading);
    manageWheelIconsBtn->setEnabledButtons(1, s.scene.online && (!themeDownloading || wheelDownloading) && !cancellingDownload && !themeDeleting && !wheelsDownloaded);
    manageWheelIconsBtn->setEnabledButtons(2, !themeDeleting && !themeDownloading);
  }

  started = s.scene.started;
}

void FrogPilotVisualsPanel::updateCarToggles() {
  std::string carParams = params.get("CarParamsPersistent");
  if (!carParams.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(carParams.data(), carParams.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    std::string carName = CP.getCarName();

    hasAutoTune = (carName == "hyundai" || carName == "toyota") && CP.getLateralTuning().which() == cereal::CarParams::LateralTuning::TORQUE;
    hasOpenpilotLongitudinal = hasLongitudinalControl(CP);
  } else {
    hasAutoTune = true;
    hasOpenpilotLongitudinal = true;
  }
}

void FrogPilotVisualsPanel::showToggles(const std::set<QString> &keys) {
  setUpdatesEnabled(false);

  for (auto &[key, toggle] : toggles) {
    toggle->setVisible(keys.find(key) != keys.end());
  }

  setUpdatesEnabled(true);
  update();
}

void FrogPilotVisualsPanel::hideToggles() {
  setUpdatesEnabled(false);

  personalizeOpenpilotOpen = false;

  for (auto &[key, toggle] : toggles) {
    bool subToggles = bonusContentKeys.find(key) != bonusContentKeys.end() ||
                      customOnroadUIKeys.find(key) != customOnroadUIKeys.end() ||
                      personalizeOpenpilotKeys.find(key) != personalizeOpenpilotKeys.end() ||
                      qolKeys.find(key) != qolKeys.end() ||
                      screenKeys.find(key) != screenKeys.end();

    toggle->setVisible(!subToggles);
  }

  setUpdatesEnabled(true);
  update();
}

void FrogPilotVisualsPanel::hideSubToggles() {
  if (personalizeOpenpilotOpen) {
    showToggles(bonusContentKeys);
  }

  setUpdatesEnabled(true);
}
