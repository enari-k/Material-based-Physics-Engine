# Material-based-Physics-Engine

Unityを描画エンジンとして利用し、物理演算ロジックをゼロから自作するプロジェクトです。

[![Zenn Article](https://img.shields.io/badge/Zenn-連載中-3EA8FF?style=for-the-badge&logo=zenn&logoColor=white)](https://zenn.dev/enari_k/articles/f21bc592f87a7a)

## 📌 概要
物理エンジンの内部構造を深く理解するため、既存の物理エンジン（PhysX等）に頼らず、数式からプログラムへと落とし込むプロセスを実践しています。

最終的には、自身の専門である**材料工学**（物質理工学院での知見）を物理パラメータや挙動に導入し、一般的な物理エンジンよりも材料特性に特化した独自の物理シミュレーションライブラリを目指しています。

## 🚀 現在の進捗 / 実装済み機能
- [x] 座標更新・速度更新の基礎
- [x] 剛体（Rigidbody）の基礎設計
- [x] 衝突検知アルゴリズムの自作
- [ ] 複数点衝突の実装
- [ ] 材料特性（弾性率、塑性変形など）の反映（開発中）

## 📚 連載記事
このリポジトリの解説は、Zennにて詳細に記事化しています。
- **[Unityを描画装置として用いて物理エンジンを自作してみた](https://zenn.dev/enari_k/articles/f21bc592f87a7a)**

## 🛠 技術スタック
- **Language:** C#
- **Platform:** Unity (Rendering / Editor)
- **Field:** Physics Simulation, Material Engineering

---
Created by [enari-K](https://github.com/enari-K)
