#pragma once
// ============================================================================
// YouTubeLoad.hpp — drop-in context-menu helper
// ----------------------------------------------------------------------------
// Adds a "Visit my Youtube channel!" entry (with a separator) to any Rack module's
// right-click menu. Just #include this header and call agmm::addGitLink(menu)
// from your Widget's appendContextMenu() override.
// ============================================================================
#include "plugin.hpp"

namespace agmm
{
    // The repo URL shown by the menu entry.
    static const char *AGMM_GIT_URL = "https://www.youtube.com/@andreas.gkogkos";

    // Append the "Visit my Youtube channel!" `.
    // Safe to call even when `module` is null (browser preview).
    inline void addYouTubeLink(Menu *menu)
    {
        if (!menu)
            return;
        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuItem(
            "Visit my Youtube channel!",
            "",
            []()
            { rack::system::openBrowser(AGMM_GIT_URL); }));
    }
} // namespace agmm