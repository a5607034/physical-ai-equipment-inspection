# 公開前チェック

1. 必ず**新規の提出専用フォルダ**で `git init` してください。`~/ros2_ws` 全体を無条件に `git add .` しないでください。
2. 公開に必要なソース：`src/my_py_pkg`, `src/my_robot_description`, `src/my_robot_bringup`, `inspection_report.py`, `spawn_equipment.sh`。ただし各パッケージに不要な過去演習や秘密情報が含まれていないか確認。
3. `README.md` と `.gitignore` をリポジトリのルートに配置。
4. シェル上で `git status --short` と `git diff --cached --name-only` を確認。
5. 可能なら `git grep -nE 'sk-[A-Za-z0-9_-]{10,}|OPENAI_API_KEY\s*=|api_key\s*=' -- ':!README.md'` を実行。検出した記述を確認（誤検出あり）。
6. `build/`, `install/`, `log/`, `.venv/`, `data/captures/`, `data/reports/` を公開しない。
7. 他者由来のGazebo 3Dモデル・世界ファイル・メッシュを再配布する場合はライセンスをチェック。
8. GitHubで `public` 設定を確認し、READMEのコマンドを第三者が読めるか確かめる。
9. 動画とPDFの提出先は課題指示に従う。GitHubにはソースコード中心でよい。
