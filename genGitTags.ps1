$gitHash = git rev-parse HEAD
$gitTag = git describe --tags --always --dirty

@"
#pragma once

#define GIT_HASH "$gitHash"
#define GIT_TAG "$gitTag"
"@ | Out-File -FilePath git_info.h -Encoding ascii