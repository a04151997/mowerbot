# 出處

這個套件不是本專案寫的，是整包收進來的第三方套件（階段 35），**沒有做任何修改**。

| 項目 | 內容 |
|------|------|
| 上游 | https://github.com/joshnewans/joy_tester |
| 作者 | Josh Newans |
| 版本 | 0.0.2，commit `dc574bc89f1d06e7a129dbb3524af55531187df0`（2024-11-16） |
| 授權 | Apache License 2.0，全文見同目錄的 `LICENSE`（依授權條款保留，不可刪） |

原本是用 `git clone` 放在這裡、自己帶著 `.git`，外層 repo 只記了一個子模組指標
（沒有 `.gitmodules`），所以 `git clone` / `git bundle` 還原之後這個資料夾是空的，
smoke test A1 會失敗。收進來時拿掉了它自己的 `.git`，檔案內容與上游該 commit 相同。
