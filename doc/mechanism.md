TILDE の動作原理
===

TILDE ではメイントピックの publish 時に PubInfo というメタ情報を `<topic名>/info/pub` に publish します。  
PubInfo は数百バイト程度のメッセージで、メイントピックを構成する入力トピックの情報が記載されます。  

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [TILDE の動作原理](#tilde-の動作原理)
    - [PubInfo](#pubinfo)
    - [PubInfo の作成](#pubinfo-の作成)
    - [オーバーヘッド](#オーバーヘッド)

<!-- markdown-toc end -->


## PubInfo

PubInfo はメイントピックの publish 時と同時に送信されるメタ情報です。
`<メイントピック名>/info/pub` に送信されます。

メッセージ定義は以下の通りです。
※ TODO: ファイル名やデータ構造はリファクタ予定

[PubInfo.msg](../src/path_info_msg/msg/PubInfo.msg)

- Header:
  - header 
  - シーケンス番号
  - 送信者情報(node 名や publisher ID)
- `output_info`: 出力トピックに関する情報
  - トピック名
  - header stamp
  - 送信時刻
- `input_infos`: 入力トピックに関する情報。下記を入力トピック分。
  - トピック名
  - header stamp
  - 受信時刻

トピック名やノード名の長さにもよりますがデータサイズは以下の通りです。送信周波数はメイントピックと同じです。

- Header + output_info: 約 80 byte + トピック名やノード名分のバイト数
- input_infos: 入力トピック一件あたり約 40 byte + トピック名のバイト数

## PubInfo の作成

※ TODO: 大体以下の事を記述する

- アプリケーションから見た TILDE
  - PubInfo を作成するのに必要な情報の蓄積や PubInfo の送信は TILDE が行なう。
  - その為、基本的にはアプリケーションで TILDE や PubInfo のことを考える必要はない。
  - ただし内部でバッファリングしている場合、メッセージを正しくトラッキングするには input_info を明示的に登録する必要がある
- PubInfo の作成
  - TILDE のカスタム create_publisher によりカスタムの Publisher である TildePublisher が作成される。
  - TildePublisher は入力情報に関するデータを持っている。
  - メイントピックの publish をフックし、メイントピックを送信すると同時に入力情報データから PubInfo を作成してPubInfo を送信する。
- input infos の紐付け
  - TILE のカスタム create_subscription により subscription コールバックがフックされる。
  - フック中の処理で TildePublisher に対して入力トピックや header stamp などの情報を登録する。

## オーバーヘッド

※ TODO: 大体以下を記載する。

- TILDE 有無時のオーバヘッド
  - autoware か demo システムを対象に TILDE 適用有無時のベンチマーク結果を記載する
  - CPU 利用率やネットワーク帯域など ← リソース分析で検討しているプロセスリソースモニタリングツールが使える?
