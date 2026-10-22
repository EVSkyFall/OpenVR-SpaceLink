;--------------------------------
; Include Modern UI

!include "MUI2.nsh"
!include "FileFunc.nsh"

;--------------------------------
; General Configuration

!define APP_VERSION "1.0.0"
!define APP_VERSION_META "1.0.0.0"
!define APP_NAME "OpenVR-SpaceLink"
!define DISPLAY_NAME "SpaceLink"

!define INSTALL_DIR "$PROGRAMFILES64\${APP_NAME}"
!ifndef FILES_DIR
!define FILES_DIR "../bin/"
!endif
!ifndef DRIVER_DIR
!define DRIVER_DIR "driver"
!endif
!ifndef LICENSE_FILE
!define LICENSE_FILE "${FILES_DIR}\LICENSE.txt"
!endif
!ifndef OUT_DIR
!define OUT_DIR "."
!endif

Name "${DISPLAY_NAME}"
OutFile "${OUT_DIR}\${APP_NAME}_Installer.exe"
InstallDir "${INSTALL_DIR}"
InstallDirRegKey HKLM "Software\${APP_NAME}\Main" ""
RequestExecutionLevel admin
ShowInstDetails show

VIProductVersion "${APP_VERSION_META}"
VIAddVersionKey /LANG=1033 "ProductName" "${DISPLAY_NAME}"
VIAddVersionKey /LANG=1033 "FileDescription" "${DISPLAY_NAME} Installer"
VIAddVersionKey /LANG=1033 "LegalCopyright" "Copyright (c) 2026 Nyabsi; SpaceLink changes Copyright (c) 2026 EVSkyFall"
VIAddVersionKey /LANG=1033 "FileVersion" "${APP_VERSION_META}"
VIAddVersionKey /LANG=1033 "ProductVersion" "${APP_VERSION}"

;--------------------------------
; Variables

Var alreadyInstalled
Var legacyInstallDir

;--------------------------------
; Interface Settings

!define MUI_ABORTWARNING

;--------------------------------
; Pages

!insertmacro MUI_PAGE_LICENSE "${LICENSE_FILE}"
!define MUI_PAGE_CUSTOMFUNCTION_PRE dirPre
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES

!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES

;--------------------------------
; Language

!insertmacro MUI_LANGUAGE "English"

;--------------------------------
; Functions

Function dirPre
    StrCmp $alreadyInstalled "true" 0 +2
        Abort
FunctionEnd

Function GetUninstallDirectory
    StrCpy $R1 $R0 1
    StrCmp $R1 '$\"' 0 +2
        StrCpy $R0 $R0 "" 1
    ${GetParent} "$R0" $R0
FunctionEnd

; Runs the overlay with the setup argument on the stack, again while it cannot reach SteamVR.
!macro RUN_SETUP_STEP_FUNCTION un
Function ${un}RunSetupStep
    Exch $R0
    Push $R1
    Push $R2
    StrCpy $R1 0
    retry:
        nsExec::ExecToLog '"$INSTDIR\OpenVR-SpaceOverride.exe" $R0'
        Pop $R2
        StrCmp $R2 0 done
        IntOp $R1 $R1 + 1
        IntCmp $R1 10 done
        DetailPrint "$R0 failed ($R2), trying again"
        Sleep 2000
        Goto retry
    done:
    Pop $R2
    Pop $R1
    Pop $R0
FunctionEnd
!macroend
!insertmacro RUN_SETUP_STEP_FUNCTION ""
!insertmacro RUN_SETUP_STEP_FUNCTION "un."

Function .onInit
    StrCpy $alreadyInstalled "false"

    ReadRegStr $R0 HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "UninstallString"
    StrCmp $R0 "" legacy
    StrCpy $alreadyInstalled "true"
    ReadRegStr $R1 HKLM "Software\${APP_NAME}\Main" ""
    StrCmp $R1 "" 0 ownpath
        Call GetUninstallDirectory
        StrCpy $R1 $R0
    ownpath:
        StrCpy $INSTDIR $R1

    legacy:
        ReadRegStr $legacyInstallDir HKLM "Software\OpenVR-SpaceOverride\Main" ""
        StrCmp $legacyInstallDir "" 0 done
        ReadRegStr $R0 HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVR-SpaceOverride" "UninstallString"
        StrCmp $R0 "" done
        Call GetUninstallDirectory
        StrCpy $legacyInstallDir $R0

    done:
FunctionEnd


;--------------------------------
; Installer Section

Section "Install" SecInstall

    StrCmp $alreadyInstalled "true" 0 noupgrade
        DetailPrint "Cleaning previous installation..."
        ExecWait '"$INSTDIR\Uninstall.exe" /S _?=$INSTDIR'
        Delete "$INSTDIR\Uninstall.exe"
    noupgrade:

    StrCmp $legacyInstallDir "" nolegacy
        DetailPrint "Replacing OpenVR-SpaceOverride in $legacyInstallDir..."
        IfFileExists "$legacyInstallDir\Uninstall.exe" 0 nolegacy
        ExecWait '"$legacyInstallDir\Uninstall.exe" /S _?=$legacyInstallDir'
    nolegacy:

    SetOutPath "$INSTDIR"

    File "${FILES_DIR}\LICENSE.txt"
	File "${FILES_DIR}\LICENSE"
	File "${FILES_DIR}\LICENSES"
	File "${FILES_DIR}\manifest.vrmanifest"
    File "${FILES_DIR}\OpenVR-SpaceOverride.exe"
    File "${FILES_DIR}\openvr_api.dll"
    File "${FILES_DIR}\icon.png"

    SetOutPath "$INSTDIR\driver"
    File /r "${DRIVER_DIR}\*"

	SetOutPath "$INSTDIR"
    Var /GLOBAL vrRuntimePath
	nsExec::ExecToStack '"$INSTDIR\OpenVR-SpaceOverride.exe" -openvrpath'
	Pop $0
	Pop $vrRuntimePath
	DetailPrint "VR runtime path: $vrRuntimePath"

    StrCmp $legacyInstallDir "" nolegacycleanup
        DetailPrint "Removing previous driver registration: $legacyInstallDir\driver"
        ExecWait '"$vrRuntimePath\bin\win64\vrpathreg.exe" removedriver "$legacyInstallDir\driver"'
        Delete "$legacyInstallDir\Uninstall.exe"
        RMDir "$legacyInstallDir"
    nolegacycleanup:
    DeleteRegKey HKLM "Software\OpenVR-SpaceOverride"
    DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVR-SpaceOverride"
    Delete "$SMPROGRAMS\OpenVR-SpaceOverride.lnk"

    WriteRegStr HKLM "Software\${APP_NAME}\Main" "" $INSTDIR
    WriteUninstaller "$INSTDIR\Uninstall.exe"

    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "DisplayName" "${DISPLAY_NAME}"
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "DisplayVersion" "${APP_VERSION}"
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "Publisher" "EVSkyFall"
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "DisplayIcon" "$INSTDIR\OpenVR-SpaceOverride.exe"
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "URLInfoAbout" "https://github.com/EVSkyFall/OpenVR-SpaceLink"
    WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "NoModify" 1
    WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "NoRepair" 1
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "UninstallString" "$\"$INSTDIR\Uninstall.exe$\""

    CreateShortCut "$SMPROGRAMS\${DISPLAY_NAME}.lnk" "$INSTDIR\OpenVR-SpaceOverride.exe"

    ; Before adddriver: adding a driver makes a running SteamVR restart its server, and a step that
    ; starts during the restart cannot connect, which once left SpaceLink unregistered as an app.
    Push "-installmanifest"
    Call RunSetupStep
    Push "-activatemultipledrivers"
    Call RunSetupStep

    ExecWait '"$vrRuntimePath\bin\win64\vrpathreg.exe" adddriver "$INSTDIR\driver"'

SectionEnd

;--------------------------------
; Uninstaller Section

Section "Uninstall"

	SetOutPath "$INSTDIR"
    Var /GLOBAL vrRuntimePath2
	nsExec::ExecToStack '"$INSTDIR\OpenVR-SpaceOverride.exe" -openvrpath'
	Pop $0
	Pop $vrRuntimePath2
	DetailPrint "VR runtime path: $vrRuntimePath2"

    Push "-removemanifest"
    Call un.RunSetupStep
    ExecWait '"$vrRuntimePath2\bin\win64\vrpathreg.exe" removedriver "$INSTDIR\driver"'
    SetOutPath "$TEMP"

    Delete "$INSTDIR\LICENSE.txt"
	Delete "$INSTDIR\LICENSE"
	Delete "$INSTDIR\LICENSES"
	Delete "$INSTDIR\manifest.vrmanifest"
    Delete "$INSTDIR\OpenVR-SpaceOverride.exe"
    Delete "$INSTDIR\openvr_api.dll"
    Delete "$INSTDIR\icon.png"
    RMDir /r "$INSTDIR\driver"

    DeleteRegKey HKLM "Software\${APP_NAME}"
    DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}"
    Delete "$SMPROGRAMS\${DISPLAY_NAME}.lnk"

    Delete "$INSTDIR\Uninstall.exe"
    RMDir "$INSTDIR"

SectionEnd
