# STM32G Motor Node Starter

Ebben a mappaban most egy indulo STM32 `FDCAN` motor node modul van, ami a handoff szerint felkeszit a kovetkezo lepesre:

- `GLOBAL_CONTROL` fogadas
- `NODE_CMD` fogadas
- `NODE_STATUS_FAST` kuldes
- `NODE_PRESENCE` kuldes
- `NODE_DIAG` kuldes

Ez nem egy teljes CubeMX projekt, hanem egy beemelheto forrasmodul. Erre azert volt szukseg, mert a mappaban eddig nem volt STM32 firmware projektstruktura, csak a handoff leiras.

## Build kornyezet

Most bekerult egy VS Code + CMake alap is, hogy a CAN-only fejlesztest rendezett kornyezetben tudjuk folytatni:

- `CMakeLists.txt`
- `CMakePresets.json`
- `cmake/gcc-arm-none-eabi.cmake`
- `../../.vscode/settings.json`
- `../../.vscode/tasks.json`
- `.clangd`

Ez a setup jelenleg egy statikus library-t fordit:

- `motor_node_can.c`

Ez szandekos, mert teljes STM32Cube projekt es linker script meg nincs a repo-ban, de a CAN modul mar kulon lefordithato lesz, ha megadod a HAL include csomagot es az ARM toolchaint.

Ehhez a preset mar a gepeden talalt Visual Studio Build Tools-os `ninja.exe`-t hasznalja, igy kulon Ninja telepites nem kell.

Alapertelmezetten `stub HAL` modban fut a standalone build. Ez azt jelenti, hogy egy minimalis helyi `fdcan.h` stubbal mar most is le tudjuk forditani a CAN modult szintaktikai ellenorzesre, akkor is, ha a teljes STM32CubeG4 csomag meg nincs telepitve.

### Szukseges kornyezeti valtozok

Windows PowerShell-ben allitsd be:

```powershell
$env:ARM_GCC_BIN_DIR="C:\ide\arm-gnu-toolchain\bin"
$env:STM32CUBE_FW_G4_DIR="C:\STM32Cube\Repository\STM32Cube_FW_G4_V1.x.x"
```

Az `ARM_GCC_BIN_DIR` abba a mappaba mutasson, ahol az `arm-none-eabi-gcc.exe` van.

Az `STM32CUBE_FW_G4_DIR` pedig a teljes STM32CubeG4 firmware package gyokerere mutasson.

Ha csak a standalone CAN modul build a cel, a `STM32CUBE_FW_G4_DIR` egyelore nem kotelezo, mert a preset alapbol `stub HAL` modban indul.

### Build inditas

Repo gyokerben:

```powershell
code .
```

VS Code-ban:

- `Terminal -> Run Task -> stm32g-node: configure`
- `Terminal -> Run Build Task`

vagy kezzel:

```powershell
cd firmware\stm32g-node
cmake --preset stm32g431-debug
cmake --build --preset stm32g431-debug
```

Valodi STM32 HAL include-okkal:

```powershell
cd firmware\stm32g-node
cmake --preset stm32g431-debug -DSTM32G_NODE_USE_STUB_HAL=OFF
cmake --build --preset stm32g431-debug
```

### VS Code extension utkozes

A hiba:

```text
You have both the Microsoft C++ (cpptools) extension and stm32-cube-clangd extension enabled.
```

ennek az oka, hogy egyszerre ket kulon IntelliSense motor probal dolgozni.

Ebben a repo-ban workspace szinten letiltottam a `cpptools` IntelliSense reszet a `.vscode/settings.json` fajlban:

- `C_Cpp.intelliSenseEngine = disabled`
- `C_Cpp.errorSquiggles = disabled`
- `C_Cpp.autocomplete = disabled`

Ha a popup ettol meg megjelenik, akkor VS Code-ban csinald ezt:

1. `Extensions`
2. `C/C++` extension
3. `Disable (Workspace)`

Igy a `stm32-cube-clangd` mar nem fog vele osszeakadni ebben a projektben.

## Fajlok

- `include/motor_node_can.h`
- `src/motor_node_can.c`

## Mire jo most

Ha a CAN/FDCAN periferiad mar ossze van kotve es a HAL handle (`FDCAN_HandleTypeDef hfdcan1`) elerheto, akkor ez a modul:

- kiszamolja a helyes CAN ID-ket `S0..S3` profil alapjan
- fogadja a `GLOBAL_CONTROL` frame-et
- fogadja a sajat `NODE_CMD = base + node_id` frame-et
- kiszamolja a cel fordulatot `base_rpm + trim_rpm`
- figyeli a sajat section bitet
- leallit `drive off / estop / section off / disable` esetben
- periodikusan kuld status, presence es diag frame-eket

## Gyors integracio

1. Masold be a fajlokat a CubeIDE projektedbe.
2. Add hozza az include path-hoz az `include` mappat.
3. A `main.c` elejen include-old:

```c
#include "motor_node_can.h"
```

4. Hozz letre egy peldanyt:

```c
extern FDCAN_HandleTypeDef hfdcan1;

static MotorNodeCan g_node;
```

5. Init utan konfigurald:

```c
MotorNodeCanConfig cfg = {
    .hfdcan = &hfdcan1,
    .node_id = 1,
    .section_index = 0,
    .profile = MOTOR_NODE_CAN_PROFILE_S0,
    .status_period_ms = 100,
    .presence_period_ms = 250,
    .diag_period_ms = 500
};

MotorNodeCan_Init(&g_node, &cfg);
HAL_FDCAN_Start(&hfdcan1);
HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
```

6. A CAN RX callbackben add at a beerkezo frame-et:

```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t flags)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    if ((flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) == HAL_OK) {
            MotorNodeCan_OnRxMessage(&g_node, &rx_header, data);
        }
    }
}
```

7. A fo ciklusban frissitsd a minta allapotot es hivd a processzt:

```c
uint32_t now = HAL_GetTick();

MotorNodeCan_SetMeasuredState(&g_node, measured_rpm, measured_pos_u16, sync_error);
MotorNodeCan_SetDiag(&g_node, bus_v, motor_a, ctrl_temp, motor_temp, fault_flags, warning_flags);
MotorNodeCan_SetSeedData(&g_node, seed_flags, blockage, slowdown, skip, doubles, singulation, population_x1k);
MotorNodeCan_Process(&g_node, now);
```

## Fontos megjegyzes

A `NODE_CMD` 4-5. byte-jat itt mar `section_mask`-kent kezeljuk a handoff szerint, nem pozicio offsetkent.

Ha kuldesre szeretned kiprobalni a legelso CAN uzenetet, akkor mar ennyi is eleg:

- `node_id = 1`
- `profile = S0`
- `MotorNodeCan_SetMeasuredState(..., 1234, 0x4000, 0);`
- `MotorNodeCan_Process(HAL_GetTick())`

Ezzel a modul ki fog kuldeni:

- `0x181` `NODE_STATUS_FAST`
- `0x141` `NODE_PRESENCE`
- `0x1C1` `NODE_DIAG` ha az ECU `diag_enable` bitje be van kapcsolva

## Kovetkezo lepes

Ha akarod, a kovetkezo korben tudok erre epitve egy konkret `STM32CubeIDE` projektvazat is csinalni `main.c`, init, callback es filter beallitasokkal.

## STM32G_node projekt build

A mostani, valodi tesztelt projekt a kovetkezo mappaban van:

- `firmware/stm32g-node/STM32G_node`

Forditas PowerShell-bol:

```powershell
cd firmware\stm32g-node\STM32G_node
cmake --build --preset Debug
```

Ha a build rendben lefut, a fontos kimeneti fajlok itt lesznek:

- `build/Debug/STM32G_node`
- `build/Debug/STM32G_node.hex`
- `build/Debug/STM32G_node.bin`

USB/DFU feltolteshez a legegyszerubb most ez:

- `STM32CubeProgrammer`
- fajl: `build/Debug/STM32G_node.bin`
- cim: `0x08000000`

Ha a firmware feltoltese utan a node rendben indul, akkor a CAN buszon meg kell jelenjen:

- `0x141` `NODE_PRESENCE`
- `0x181` `NODE_STATUS_FAST`

### Aktualis CAN pinout

A stabil boot miatt a CAN mar nem a `PB8/PB9` paron van, mert a `PB8` ezen a tokozason `BOOT0` is.

Az aktualis firmware szerint:

- `PA11` -> `FDCAN1_RX`
- `PA12` -> `FDCAN1_TX`

Ez azt jelenti, hogy ezen a kiosztason a panel sajat USB csatlakozojat most nem hasznaljuk alkalmazas oldali `USB CDC`-re.

Ha ezek nem jelennek meg, akkor eloszor mindig azt ellenorizd:

- a megfelelo `.bin` lett-e feltoltve
- nem maradt-e DFU/bootloader modban a panel
- 250 kbit/s-on figyeled-e a buszt
