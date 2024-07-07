// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// This file is part of g2o.
//
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with g2o.  If not, see <http://www.gnu.org/licenses/>.

#include "abstract_properties_widget.h"

#include <QLineEdit>
#include <cassert>
#include <iostream>
#include <memory>

#include "g2o/stuff/property.h"

AbstractPropertiesWidget::AbstractPropertiesWidget(QWidget* parent)
    : QDialog(parent)

{
  setupUi(this);
  connect(btnApply, &QPushButton::clicked, this,
          &AbstractPropertiesWidget::on_btnApply_clicked);
  connect(btnOK, &QPushButton::clicked, this,
          &AbstractPropertiesWidget::on_btnOK_clicked);
}

void AbstractPropertiesWidget::updateDisplayedProperties() {
  tableWidget->clear();
  propNames_.clear();

  tableWidget->setColumnCount(2);

  QStringList horizontalHeaders;
  horizontalHeaders.append("Name");
  horizontalHeaders.append("Value");
  tableWidget->setHorizontalHeaderLabels(horizontalHeaders);

  tableWidget->verticalHeader()->hide();

  const g2o::PropertyMap* properties = propertyMap();
  if (!properties) return;
  tableWidget->setRowCount(properties->size());

  int r = 0;
  for (auto it = properties->begin(); it != properties->end(); ++it, ++r) {
    auto* textItem = new QTableWidgetItem;
    textItem->setText(QString::fromStdString(humanReadablePropName(it->first)));
    textItem->setFlags(textItem->flags() & ~Qt::ItemIsEditable);
    tableWidget->setItem(r, 0, textItem);
    propNames_.push_back(it->first);

    if (dynamic_cast<g2o::Property<bool>*>(it->second.get())) {
      auto* prop = static_cast<g2o::Property<bool>*>(it->second.get());
      auto* checkItem = new QTableWidgetItem;
      checkItem->setText("enabled");
      checkItem->setFlags(checkItem->flags() | Qt::ItemIsUserCheckable);
      if (prop->value())
        checkItem->setCheckState(Qt::Checked);
      else
        checkItem->setCheckState(Qt::Unchecked);
      tableWidget->setItem(r, 1, checkItem);
    } else {
      auto* editor = new QLineEdit(tableWidget);
      editor->setText(QString::fromStdString(it->second->toString()));
      if (dynamic_cast<g2o::Property<int>*>(it->second.get())) {
        editor->setValidator(new QIntValidator(editor));
      } else if (dynamic_cast<g2o::Property<float>*>(it->second.get()) ||
                 dynamic_cast<g2o::Property<double>*>(it->second.get())) {
        editor->setValidator(new QDoubleValidator(editor));
      }
      tableWidget->setCellWidget(r, 1, editor);
    }
  }
  tableWidget->resizeColumnToContents(0);
}

void AbstractPropertiesWidget::applyProperties() {
  assert(tableWidget->rowCount() == (int)propNames_.size());
  const g2o::PropertyMap* properties = propertyMap();
  if (!properties) return;
  for (int r = 0; r < tableWidget->rowCount(); ++r) {
    const std::string& propName = propNames_[r];
    // HACK cast away the const
    std::shared_ptr<g2o::BaseProperty> baseProp =
        std::const_pointer_cast<g2o::BaseProperty>(
            properties->getProperty<g2o::BaseProperty>(propName));
    if (!baseProp) continue;

    if (dynamic_cast<g2o::Property<bool>*>(baseProp.get())) {
      auto* prop = static_cast<g2o::Property<bool>*>(baseProp.get());
      QTableWidgetItem* checkItem = tableWidget->item(r, 1);
      prop->setValue(checkItem->checkState() == Qt::Checked);
    } else {
      auto* editor = dynamic_cast<QLineEdit*>(tableWidget->cellWidget(r, 1));
      bool status = baseProp->fromString(editor->text().toStdString());
      if (!status) {
        std::cerr << "Warning: unable to set property " << baseProp->name()
                  << '\n';
      }
    }
  }
}

void AbstractPropertiesWidget::on_btnApply_clicked() { applyProperties(); }

void AbstractPropertiesWidget::on_btnOK_clicked() {
  applyProperties();
  close();
}

std::string AbstractPropertiesWidget::humanReadablePropName(
    const std::string& propertyName) const {
  return propertyName;
}
