"""
pengendali_track_kanan.py
-------------------------
Contoh program ROS 2 (Python) untuk menggerakkan mobil di Gazebo Harmonic melewati
track kanan lewat topic /cmd_vel (geometry_msgs/msg/Twist).

Prinsip kerja (open-loop, berbasis waktu):
  1. Jalur ditulis sebagai daftar gerakan (JALUR): lurus, rotasi di tempat,
     lengkung, dan tunggu.
  2. Tiap gerakan diubah menjadi tiga nilai: kecepatan maju, kecepatan putar,
     dan durasi.
  3. Sebuah timer berjalan terus dan mengirim perintah dari gerakan yang sedang
     aktif ke /cmd_vel, sampai seluruh jalur selesai.

Satuan: jarak dalam meter perintah, sudut dalam derajat perintah. Tidak ada faktor
skala atau koreksi tersembunyi, jadi semua angka pada JALUR dipakai apa adanya.
Rotasi kiri = angular.z positif, rotasi kanan = angular.z negatif.
Kata atas/bawah/kiri/kanan pada keterangan = arah pada tampilan atas Gazebo,
bukan arah belok mobil.

Angka pada JALUR disetel manual untuk robot dan world milik pembuatnya. Jika robot
atau world berbeda, kalibrasi ulang jarak dan sudutnya.
"""

import math
import time
from bisect import bisect_right
from itertools import accumulate
from typing import Any, NamedTuple

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

# ------------------------------ Parameter dasar ------------------------------
KECEPATAN_MAJU = 3.0                   # m/s
KECEPATAN_PUTAR = math.radians(180)    # rad/s (180 derajat per detik)
SELANG_KIRIM = 0.02                    # detik (50 Hz)


# ------------------------------- Tipe data ----------------------------------
class Gerak(NamedTuple):
    """Satu gerakan pada jalur (kecepatan dan durasinya belum dihitung)."""
    jenis: str
    nilai: Any
    ket: str = ''


class Perintah(NamedTuple):
    """Hasil hitung satu gerakan: apa yang dikirim ke /cmd_vel dan berapa lama."""
    ket: str
    v: float            # linear.x (m/s)
    w: float            # angular.z (rad/s)
    durasi: float       # detik


# --------------------- Fungsi penulis gerakan (untuk JALUR) ------------------
def lurus(jarak: float, ket: str = '') -> Gerak:
    """Bergerak lurus sejauh `jarak` meter perintah."""
    return Gerak('lurus', jarak, ket)


def rotasi_kiri(derajat: float, ket: str = '') -> Gerak:
    """Berputar di tempat ke kiri (berlawanan arah jarum jam)."""
    return Gerak('rotasi', +derajat, ket)


def rotasi_kanan(derajat: float, ket: str = '') -> Gerak:
    """Berputar di tempat ke kanan (searah jarum jam)."""
    return Gerak('rotasi', -derajat, ket)


def lengkung_kiri(radius: float, derajat: float, ket: str = '') -> Gerak:
    """Bergerak melengkung ke kiri; kecepatan maju dan putar aktif bersamaan."""
    return Gerak('lengkung', (radius, +derajat), ket)


def lengkung_kanan(radius: float, derajat: float, ket: str = '') -> Gerak:
    """Bergerak melengkung ke kanan dengan jari-jari `radius` meter perintah."""
    return Gerak('lengkung', (radius, -derajat), ket)


def tunggu(detik: float, ket: str = '') -> Gerak:
    """Diam sejenak (bisa disisipkan di antara gerakan untuk meredam goyangan)."""
    return Gerak('tunggu', detik, ket)


# ------------------------------ Jalur track kanan ----------------------------
TAHAP_MENUJU_LORONG = [
    tunggu(1.0,                  'Tunggu koneksi ke /cmd_vel'),
    rotasi_kanan(87,             'Menghadap kanan'),   # hapus jika sudah menghadap kanan
    lurus(8.40,                  'Maju ke depan mulut lorong'),
    rotasi_kiri(120,             'Menghadap atas, siap masuk lorong'),
]

TAHAP_MASUK_LORONG = [
    lurus(8.13,                  'Masuk lorong hingga bagian yang melebar'),
    rotasi_kanan(117,            'Menghadap kanan'),
    lurus(1.90,                  'Bergeser ke kanan'),
    rotasi_kiri(115,             'Menghadap atas'),
    lurus(3.80,                  'Naik hingga ujung lorong sempit'),
]

TAHAP_SISI_KIRI = [
    rotasi_kiri(113,             'Menghadap kiri'),
    lurus(7.00,                  'Maju ke kiri hingga pojok'),
    rotasi_kanan(115,            'Menghadap atas'),
    lurus(14.00,                 'Naik menyusuri lorong kiri'),
]

TAHAP_LORONG_ATAS = [
    rotasi_kanan(115,            'Menghadap kanan'),
    lurus(16.00,                 'Maju menyusuri lorong atas'),
    lengkung_kanan(4.50, 180,    'Tikungan setengah lingkaran (U-turn ke kanan)'),
    tunggu(1.0,                  'Redam gerak setelah tikungan'),
    rotasi_kanan(8,              'Koreksi arah setelah tikungan'),
]

TAHAP_MENUJU_RUANGAN = [
    lurus(6.70,                  'Maju ke kiri di bawah tikungan'),
    rotasi_kiri(120,             'Menghadap bawah'),
    lurus(3.50,                  'Turun'),
    rotasi_kiri(117,             'Menghadap kanan'),
    lurus(8.25,                  'Maju ke kanan hingga tepat di atas bukaan ruangan'),
    rotasi_kanan(115,            'Menghadap bawah'),
    lurus(5.54,                  'Masuk ruangan'),
]

TAHAP_MENUJU_FINISH = [
    rotasi_kiri(115,             'Menghadap kanan'),
    lurus(2.62,                  'Bergeser ke kanan, tepat di atas kanal sempit'),
    rotasi_kanan(117,            'Menghadap bawah'),
    lurus(6.70,                  'Turun lewat kanal sempit hingga lorong paling bawah'),
    rotasi_kanan(120,            'Menghadap kiri'),
    lurus(7.38,                  'Maju ke kiri hingga tulisan FINISH, lalu berhenti'),
]

# Urutan tahap = urutan jalan mobil dari titik awal sampai FINISH
JALUR = (TAHAP_MENUJU_LORONG + TAHAP_MASUK_LORONG + TAHAP_SISI_KIRI
         + TAHAP_LORONG_ATAS + TAHAP_MENUJU_RUANGAN + TAHAP_MENUJU_FINISH)


# ------------------- Perhitungan kecepatan dan durasi per jenis --------------
def _hitung_lurus(jarak):
    return KECEPATAN_MAJU, 0.0, jarak / KECEPATAN_MAJU


def _hitung_rotasi(derajat):
    arah = 1.0 if derajat > 0 else -1.0
    durasi = math.radians(abs(derajat)) / KECEPATAN_PUTAR
    return 0.0, arah * KECEPATAN_PUTAR, durasi


def _hitung_lengkung(nilai):
    radius, derajat = nilai
    arah = 1.0 if derajat > 0 else -1.0
    w = KECEPATAN_MAJU / radius            # pada gerak melingkar: w = v / radius
    durasi = math.radians(abs(derajat)) / w
    return KECEPATAN_MAJU, arah * w, durasi


def _hitung_tunggu(detik):
    return 0.0, 0.0, detik


PENGHITUNG = {
    'lurus': _hitung_lurus,
    'rotasi': _hitung_rotasi,
    'lengkung': _hitung_lengkung,
    'tunggu': _hitung_tunggu,
}


def bangun_jadwal(jalur):
    """Ubah daftar Gerak menjadi daftar Perintah (kecepatan + durasi)."""
    jadwal = []
    for gerak in jalur:
        try:
            hitung = PENGHITUNG[gerak.jenis]
        except KeyError:
            raise ValueError(f'Jenis gerak tidak dikenal: {gerak.jenis}') from None
        v, w, durasi = hitung(gerak.nilai)
        jadwal.append(Perintah(gerak.ket, v, w, durasi))
    return jadwal


# ----------------------------------- Node ------------------------------------
class PengendaliTrackKanan(Node):
    def __init__(self):
        super().__init__('pengendali_track_kanan')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.jadwal = bangun_jadwal(JALUR)
        # batas_waktu[i] = detik (sejak mulai) saat gerakan ke-i berakhir
        self.batas_waktu = list(accumulate(p.durasi for p in self.jadwal))

        self.nomor_aktif = -1       # -1 = belum ada gerakan yang berjalan
        self.ns_awal = None         # diisi pada tick pertama (aman untuk use_sim_time)
        self.tuntas = False

        total = self.batas_waktu[-1] if self.batas_waktu else 0.0
        self.get_logger().info(
            f'{len(self.jadwal)} langkah, estimasi total {total:.1f} detik')
        self.timer = self.create_timer(SELANG_KIRIM, self.kirim_perintah_aktif)

    def _kirim(self, v, w):
        pesan = Twist()
        pesan.linear.x = v
        pesan.angular.z = w
        self.pub_cmd.publish(pesan)

    def kirim_berhenti(self):
        """Kirim perintah berhenti (semua kecepatan 0)."""
        self._kirim(0.0, 0.0)

    def kirim_perintah_aktif(self):
        ns = self.get_clock().now().nanoseconds
        if self.ns_awal is None:
            self.ns_awal = ns
        t = (ns - self.ns_awal) / 1e9

        # gerakan aktif = gerakan pertama yang batas waktunya belum terlewati
        nomor = bisect_right(self.batas_waktu, t)

        if nomor >= len(self.jadwal):       # seluruh jalur sudah habis
            self.kirim_berhenti()
            self.timer.cancel()
            self.tuntas = True
            self.get_logger().info('Selesai. Mobil berhenti.')
            return

        if nomor != self.nomor_aktif:       # catat hanya saat gerakan berganti
            self.nomor_aktif = nomor
            self.get_logger().info(
                f'[{nomor + 1}/{len(self.jadwal)}] {self.jadwal[nomor].ket}')

        aktif = self.jadwal[nomor]
        self._kirim(aktif.v, aktif.w)


def main(args=None):
    # SignalHandlerOptions.NO: Ctrl+C menjadi KeyboardInterrupt biasa, sehingga
    # perintah berhenti masih bisa dikirim sebelum program keluar.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    pengendali = PengendaliTrackKanan()
    try:
        while rclpy.ok() and not pengendali.tuntas:
            rclpy.spin_once(pengendali, timeout_sec=0.1)
    except KeyboardInterrupt:
        pengendali.get_logger().warn('Dihentikan manual (Ctrl+C).')
    finally:
        if rclpy.ok():
            pengendali.kirim_berhenti()
            time.sleep(0.2)             # beri waktu agar pesan berhenti terkirim
            pengendali.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()