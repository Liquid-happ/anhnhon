import zipfile
import xml.etree.ElementTree as ET

def docx_to_text(docx_path):
    with zipfile.ZipFile(docx_path) as z:
        xml_content = z.read('word/document.xml')
        root = ET.fromstring(xml_content)
        paragraphs = []
        for paragraph in root.iter('{http://schemas.openxmlformats.org/wordprocessingml/2006/main}p'):
            texts = [node.text for node in paragraph.iter('{http://schemas.openxmlformats.org/wordprocessingml/2006/main}t') if node.text]
            if texts:
                paragraphs.append(''.join(texts))
            else:
                paragraphs.append('') # keep empty lines if needed
        return '\n'.join(paragraphs)

if __name__ == '__main__':
    path = 'Cấu_trúc_tổ_chức_lại_mã_nguồn_firmware_BMS_Vilas126.docx'
    content = docx_to_text(path)
    with open('docx_content.txt', 'w', encoding='utf-8') as f:
        f.write(content)
    print("Done extraction to docx_content.txt")
