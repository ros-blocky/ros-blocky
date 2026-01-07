


!macro customInstallMode
  SetDetailsView show
!macroend


!macro customInstall
  nsExec::ExecToStack 'cmd /c where pixi'
  Pop $0  ; exit code
  Pop $1  ; output path

  ${If} $0 == 0
      DetailPrint "Pixi already installed at: $1"
      Goto install_pixi_ws
  ${EndIf}

  StrCpy $0 "$INSTDIR\resources\pixi.msi"
  ExecWait 'msiexec /i "$0" /qb' $1
  ${If} $1 == 0
    DetailPrint "Pixi installed successfully."
  ${Else}
    MessageBox MB_OK|MB_ICONEXCLAMATION "Pixi installation failed or was cancelled. The application may not work correctly."
    Goto done
  ${EndIf}

  install_pixi_ws:
  ; source from extraResources (in resources folder)
  StrCpy $0 "$INSTDIR\resources\pixi_ws"

  ; destination
  StrCpy $1 "C:\pixi_ws"

  ; handle already existing folder
  IfFileExists "$1" 0 +3
    MessageBox MB_YESNO|MB_ICONQUESTION "$1 already exists. Override it?" IDYES +2 IDNO skip_copy
    RMDir /r "$1"

    ; create folder
    CreateDirectory "$1"

    ;nsExec::ExecToLog 'robocopy "$0" "$1" /E /R:3 /W:1'
    ;Pop $2
    CopyFiles "$0\*" "$1"

    Goto done

  skip_copy:
    DetailPrint "Skipping pixi_ws copy."

  done:
!macroend

