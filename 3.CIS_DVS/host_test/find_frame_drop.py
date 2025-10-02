import os
import re
import cv2

f_name= input("dat folder name (inside ./bin_files):")
folder_path = "./bin_files/"+f_name  
pattern = re.compile(r"(\d+)_f(\d{1,3})_t(\d+)\.png")

# {순서1: (순서2(f), 파일명)} 저장
file_map = {}

for entry in os.scandir(folder_path):
    if not entry.is_file():
        continue
    match = pattern.match(entry.name)
    if match:
        seq1 = int(match.group(1))
        f_val = int(match.group(2))
        file_map[seq1] = (f_val, entry.name)

sorted_keys = sorted(file_map.keys())

err_cnt = 0
for i in range(1, len(sorted_keys)):
    prev_seq1 = sorted_keys[i - 1]
    curr_seq1 = sorted_keys[i]

    if curr_seq1 != prev_seq1 + 1:
        continue  # 순서1이 건너뛴 경우는 무시

    prev_f, prev_fname = file_map[prev_seq1]
    curr_f, curr_fname = file_map[curr_seq1]
    expected_f = (prev_f + 1) % 256

    if curr_f != expected_f:
        err_cnt += 1
        print(f"\n[!] f 값 불연속 감지: f{prev_f:03} → f{curr_f:03} (f{expected_f:03} 누락)")
        print(f"    앞 파일: {prev_fname}")
        print(f"    문제 파일: {curr_fname}")

        # 이미지 파일 경로
        prev_img_path = os.path.join(folder_path, prev_fname)
        curr_img_path = os.path.join(folder_path, curr_fname)

        # 이미지 로딩
        prev_img = cv2.imread(prev_img_path, cv2.IMREAD_GRAYSCALE)
        curr_img = cv2.imread(curr_img_path, cv2.IMREAD_GRAYSCALE)

        # 이미지 표시
        cv2.imshow("Prev Image", prev_img)
        cv2.imshow("Current Image", curr_img)

        print("이미지가 열렸습니다. 아무 키나 누르면 다음 프레임드랍으로 넘어갑니다.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
if(err_cnt==0):
	print("프레임 순서가 정상입니다.")

