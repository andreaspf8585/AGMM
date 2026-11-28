#pragma once
// ============================================================================
// GitLoad.hpp — drop-in context-menu helper
// ----------------------------------------------------------------------------
// Adds a "Visit my git repo!" entry (with a separator) to any Rack module's
// right-click menu. Just #include this header and call agmm::addGitLink(menu)
// from your Widget's appendContextMenu() override.
// ============================================================================
#include "plugin.hpp"

namespace agmm
{
    // The repo URL shown by the menu entry.
    static const char *AGMM_GIT_URL = "https://github.com/andreaspf8585/AGMM";

    // Append the "Visit my git repo!" item to `menu`.
    // Safe to call even when `module` is null (browser preview).
    inline void addGitLink(Menu *menu)
    {
        if (!menu)
            return;
        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuItem(
            "Visit my git repo!",
            "",
            []()
            { rack::system::openBrowser(AGMM_GIT_URL); }));
    }
} // namespace agmm