# UI fonts

The overlay compiles these files into the executable at build time (`cmake/EmbedFile.cmake`), so it never loads them from disk or the network.

| Files | Font | License | Source |
| --- | --- | --- | --- |
| `geist/Geist-Regular.ttf`, `Geist-Medium.ttf`, `Geist-SemiBold.ttf`, `GeistMono-Regular.ttf` | Geist, Geist Mono | SIL Open Font License 1.1, `geist/OFL.txt` | [google/fonts](https://github.com/google/fonts) `ofl/geist`, `ofl/geistmono` |
| `phosphor/Phosphor.ttf` | Phosphor Icons 2.1.2, regular weight | MIT, `phosphor/LICENSE` | npm package `@phosphor-icons/web` 2.1.2 |

The Geist files come from google/fonts commit `2eb0b48d5f760f62e286216f0859a8c540dbc1bd`. Its variable fonts were pinned to the single weights the UI uses with fontTools' `instancer`, then hinting and the OpenType layout and variation tables were removed, since Dear ImGui's stb_truetype loader reads none of them. Every character each font covers was kept. The license reserves no font name, so the files keep their names.

The Phosphor font was cut down with fontTools' subsetter to 52 icons, which include every icon in `namespace Icon` of `src/overlay/UserInterface.cpp`.

Text in scripts these fonts lack, such as the Korean, Japanese or Chinese error messages Windows writes, is drawn with fonts that ship with Windows (Segoe UI, Malgun Gothic, Yu Gothic or Meiryo, Microsoft YaHei). They are read from the Windows font folder when the UI starts and skipped when missing; nothing of them is bundled (`src/overlay/SystemFonts.cpp`).
